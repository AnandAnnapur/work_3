#include "mavlink_bridge.h"
#include <iostream>
#include <unistd.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <arpa/inet.h>

// ---------------- Constructor ----------------
MavlinkBridge::MavlinkBridge(const std::string &server_ip, int server_port, DroneManager &mgr)
    : manager(mgr), running_(false) {
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) { perror("socket"); exit(1); }

    // 1️⃣ Bind to ephemeral port (client)
    sockaddr_in local_addr{};
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(0); // let OS choose free port
    local_addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock_, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
        perror("bind"); ::close(sock_); exit(1);
    }

    // 2️⃣ Router (server) address
    memset(&server_addr_, 0, sizeof(server_addr_));
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(server_port);
    if (!inet_aton(server_ip.c_str(), &server_addr_.sin_addr)) {
        std::cerr << "Invalid server IP\n"; ::close(sock_); exit(1);
    }

    running_ = true;
    recv_thread_ = std::thread(&MavlinkBridge::recv_loop, this);
    hb_thread_   = std::thread(&MavlinkBridge::hb_loop, this);

    std::cout << "[BRIDGE] Client started, sending to "
              << server_ip << ":" << server_port << std::endl;
}

// ---------------- Destructor ----------------
MavlinkBridge::~MavlinkBridge() {
    running_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();
    if (hb_thread_.joinable()) hb_thread_.join();
    ::close(sock_);
}

// ---------------- Helpers ----------------
int MavlinkBridge::send_to_drone(const sockaddr_in &, const uint8_t *buf, size_t len) {
    return sendto(sock_, buf, (int)len, 0, (const struct sockaddr*)&server_addr_, sizeof(server_addr_));
}

void MavlinkBridge::track_pending_command(int sysid, int command_id) {
    std::lock_guard<std::mutex> lock(ack_mtx);
    pending_acks.insert({sysid, command_id});
}

std::string MavlinkBridge::ack_result_to_string(int result) {
    switch (result) {
        case MAV_RESULT_ACCEPTED: return "ACCEPTED";
        case MAV_RESULT_TEMPORARILY_REJECTED: return "TEMP REJECTED";
        case MAV_RESULT_DENIED: return "DENIED";
        case MAV_RESULT_UNSUPPORTED: return "UNSUPPORTED";
        case MAV_RESULT_FAILED: return "FAILED";
        case MAV_RESULT_IN_PROGRESS: return "IN PROGRESS";
        default: return "UNKNOWN";
    }
}

// ---------------- High-level wrappers ----------------
void MavlinkBridge::request_stream(int sysid) {
    auto snap = snapshot_status();
    auto it = snap.find(sysid);
    if (it == snap.end() || !it->second.has_addr) return;

    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(
        255, MAV_COMP_ID_MISSIONPLANNER, &msg,
        sysid, MAV_COMP_ID_AUTOPILOT1,
        MAV_DATA_STREAM_ALL, 10, 1
    );
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buf, &msg);
    send_to_drone(server_addr_, buf, len);

    track_pending_command(sysid, MAV_CMD_SET_MESSAGE_INTERVAL);
}

void MavlinkBridge::set_mode_guided(int sysid) {
    auto snap = snapshot_status();
    auto it = snap.find(sysid);
    if (it == snap.end() || !it->second.has_addr) return;

    mavlink_message_t msg;
    uint8_t base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    uint32_t custom_mode = 4; // GUIDED

    mavlink_msg_set_mode_pack(
        255, MAV_COMP_ID_MISSIONPLANNER, &msg,
        sysid, base_mode, custom_mode
    );
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buf, &msg);
    send_to_drone(server_addr_, buf, len);

    track_pending_command(sysid, MAV_CMD_DO_SET_MODE);
}

void MavlinkBridge::arm(int sysid) {
    auto snap = snapshot_status();
    auto it = snap.find(sysid);
    if (it == snap.end() || !it->second.has_addr) return;

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        255, MAV_COMP_ID_MISSIONPLANNER, &msg,
        sysid, MAV_COMP_ID_AUTOPILOT1,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1,0,0,0,0,0,0
    );
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buf, &msg);
    send_to_drone(server_addr_, buf, len);

    track_pending_command(sysid, MAV_CMD_COMPONENT_ARM_DISARM);
}

void MavlinkBridge::rtl(int sysid) {
    auto snap = snapshot_status();
    auto it = snap.find(sysid);
    if (it == snap.end() || !it->second.has_addr) return;

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        255, MAV_COMP_ID_MISSIONPLANNER, &msg,
        sysid, MAV_COMP_ID_AUTOPILOT1,
        MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,0,0,0,0,0,0,0
    );
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buf, &msg);
    send_to_drone(server_addr_, buf, len);

    track_pending_command(sysid, MAV_CMD_NAV_RETURN_TO_LAUNCH);
}

void MavlinkBridge::send_waypoint(int sysid, double lat, double lon, double alt) {
    auto snap = snapshot_status();
    auto it = snap.find(sysid);
    if (it == snap.end() || !it->second.has_addr) return;

    mavlink_message_t msg;
    uint16_t type_mask =
        POSITION_TARGET_TYPEMASK_VX_IGNORE |
        POSITION_TARGET_TYPEMASK_VY_IGNORE |
        POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        POSITION_TARGET_TYPEMASK_AX_IGNORE |
        POSITION_TARGET_TYPEMASK_AY_IGNORE |
        POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    mavlink_msg_set_position_target_global_int_pack(
        255, MAV_COMP_ID_MISSIONPLANNER, &msg,
        sysid, MAV_COMP_ID_AUTOPILOT1,
        0, MAV_FRAME_GLOBAL_RELATIVE_ALT,
        type_mask,
        static_cast<int32_t>(lat * 1e7),
        static_cast<int32_t>(lon * 1e7),
        static_cast<float>(alt),
        0,0,0, 0,0,0, 0,0
    );
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buf, &msg);
    send_to_drone(server_addr_, buf, len);

    track_pending_command(sysid, MAV_CMD_NAV_WAYPOINT);
}

void MavlinkBridge::takeoff(int sysid, float altitude) {
    auto snap = snapshot_status();
    auto it = snap.find(sysid);
    if (it == snap.end() || !it->second.has_addr) {
        std::cout << "[TAKEOFF] no addr for sysid=" << sysid << ", skipping\n";
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        255, MAV_COMP_ID_MISSIONPLANNER, &msg,
        sysid, MAV_COMP_ID_AUTOPILOT1,
        MAV_CMD_NAV_TAKEOFF,
        1, 0,0,0,0, 0,0, altitude
    );
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buf, &msg);
    send_to_drone(server_addr_, buf, len);

    track_pending_command(sysid, MAV_CMD_NAV_TAKEOFF);
    std::cout << "[SEND] sysid=" << sysid << " -> TAKEOFF to " << altitude << "m\n";
}

void MavlinkBridge::circle(int sysid, double lat, double lon, double alt, int turns, float radius) {
    auto snap = snapshot_status();
    auto it = snap.find(sysid);
    if (it == snap.end() || !it->second.has_addr) {
        std::cout << "[CIRCLE] no addr for sysid=" << sysid << ", skipping\n";
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_mission_item_int_pack(
        255, MAV_COMP_ID_MISSIONPLANNER, &msg,
        sysid, MAV_COMP_ID_AUTOPILOT1,
        0, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MAV_CMD_NAV_LOITER_TURNS,
        2, 1,
        turns, radius,
        0,0,
        static_cast<int32_t>(lat * 1e7),
        static_cast<int32_t>(lon * 1e7),
        static_cast<float>(alt),
        MAV_MISSION_TYPE_MISSION
    );

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buf, &msg);
    send_to_drone(server_addr_, buf, len);

    track_pending_command(sysid, MAV_CMD_NAV_LOITER_TURNS);
    std::cout << "[SEND] sysid=" << sysid
              << " -> LOITER turns=" << turns
              << " radius=" << radius
              << " at (" << lat << "," << lon << "," << alt << ")\n";
}

void MavlinkBridge::reposition(int sysid, double lat, double lon, double alt, double v) {
    auto snap = snapshot_status();
    auto it = snap.find(sysid);
    if (it == snap.end() || !it->second.has_addr) {
        std::cout << "[REPOSITION] no addr for sysid=" << sysid << ", skipping\n";
        return;
    }

    mavlink_message_t msg;

    // ---------- 1) Send DO_CHANGE_SPEED and wait for ACK ----------
    // Track and send
    track_pending_command(sysid, MAV_CMD_DO_CHANGE_SPEED);
    mavlink_msg_command_long_pack(
        255, MAV_COMP_ID_MISSIONPLANNER, &msg,
        sysid, MAV_COMP_ID_AUTOPILOT1,
        MAV_CMD_DO_CHANGE_SPEED,
        0,                                 // confirmation
        1,                                 // param1: 1 = groundspeed
        static_cast<float>(v),             // param2: speed (m/s)
        -1, 0, 0, 0, 0                     // param3..7 unused
    );
    {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        int len = mavlink_msg_to_send_buffer(buf, &msg);
        send_to_drone(server_addr_, buf, len);
    }
    std::cout << "[SEND] sysid=" << sysid << " -> DO_CHANGE_SPEED to " << v << " m/s\n";

    // wait for ACK (2s)
    int ack_result = wait_for_ack(sysid, MAV_CMD_DO_CHANGE_SPEED, 2000);

    // retry once on timeout
    if (ack_result == -1) {
        std::cout << "[REPOSITION] no ACK for DO_CHANGE_SPEED, retrying...\n";
        track_pending_command(sysid, MAV_CMD_DO_CHANGE_SPEED);
        mavlink_msg_command_long_pack(
            255, MAV_COMP_ID_MISSIONPLANNER, &msg,
            sysid, MAV_COMP_ID_AUTOPILOT1,
            MAV_CMD_DO_CHANGE_SPEED,
            0, 1, static_cast<float>(v), -1, 0, 0, 0, 0
        );
        {
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            int len = mavlink_msg_to_send_buffer(buf, &msg);
            send_to_drone(server_addr_, buf, len);
        }
        ack_result = wait_for_ack(sysid, MAV_CMD_DO_CHANGE_SPEED, 2000);
    }

    // If DO_CHANGE_SPEED not accepted, fallback to PARAM_SET ("WPNAV_SPEED")
    if (ack_result != MAV_RESULT_ACCEPTED) {
        std::cerr << "[REPOSITION] DO_CHANGE_SPEED not accepted (result="
                  << ack_result << "). Falling back to PARAM_SET WPNAV_SPEED.\n";

        // Send PARAM_SET (WPNAV_SPEED in cm/s)
        float wpnav_speed_cm = static_cast<float>(v * 100.0);
        // Optionally nudge value to force acceptance if same as last:
        static float last_wpnav_speed = -1.0f;
        if (std::abs(wpnav_speed_cm - last_wpnav_speed) < 1e-3f) {
            wpnav_speed_cm += 1.0f; // nudge by 1 cm/s
        }
        last_wpnav_speed = wpnav_speed_cm;

        mavlink_msg_param_set_pack(
            255, MAV_COMP_ID_MISSIONPLANNER, &msg,
            sysid, MAV_COMP_ID_AUTOPILOT1,
            "WPNAV_SPEED", wpnav_speed_cm, MAV_PARAM_TYPE_REAL32
        );
        {
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            int len = mavlink_msg_to_send_buffer(buf, &msg);
            send_to_drone(server_addr_, buf, len);
        }
        // small pause to let autopilot apply it
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // ---------- 2) Send position target ----------
    uint16_t type_mask =
        POSITION_TARGET_TYPEMASK_VX_IGNORE |
        POSITION_TARGET_TYPEMASK_VY_IGNORE |
        POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        POSITION_TARGET_TYPEMASK_AX_IGNORE |
        POSITION_TARGET_TYPEMASK_AY_IGNORE |
        POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
        // only position is active

    mavlink_msg_set_position_target_global_int_pack(
        255, MAV_COMP_ID_MISSIONPLANNER, &msg,
        0, // time_boot_ms
        sysid, MAV_COMP_ID_AUTOPILOT1,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask,
        static_cast<int32_t>(lat * 1e7),
        static_cast<int32_t>(lon * 1e7),
        static_cast<float>(alt),
        0, 0, 0,   // vx,vy,vz ignored
        0, 0, 0,   // afx,afy,afz ignored
        0, 0       // yaw,yaw_rate ignored
    );
    {
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        int len = mavlink_msg_to_send_buffer(buf, &msg);
        send_to_drone(server_addr_, buf, len);
    }

    std::cout << "[SEND] sysid=" << sysid
              << " -> REPOSITION to (" << lat << ", " << lon << ", " << alt
              << ") at requested speed " << v << " m/s\n";
}


// ---------------- Ack Waiting ----------------
int MavlinkBridge::wait_for_ack(int sysid, int command, int timeout_ms) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    std::unique_lock<std::mutex> lk(cv_mtx);
    while (std::chrono::steady_clock::now() < deadline) {
        if (last_ack_sysid == sysid && last_ack_command == command) {
            return last_ack_result;
        }
        cv.wait_until(lk, deadline);
    }
    return -1;
}

// ---------------- Snapshot ----------------
std::map<int, DroneStatus> MavlinkBridge::snapshot_status() {
    std::lock_guard<std::mutex> lk(status_mtx);
    return drone_status;
}

// ---------------- Heartbeat Loop ----------------
void MavlinkBridge::hb_loop() {
    while (running_) {
        mavlink_message_t hb;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_heartbeat_pack(
            255, MAV_COMP_ID_MISSIONPLANNER, &hb,
            MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, 0
        );
        int len = mavlink_msg_to_send_buffer(buf, &hb);
        sendto(sock_, buf, len, 0, (struct sockaddr*)&server_addr_, sizeof(server_addr_));
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// ---------------- Receive Loop ----------------
void MavlinkBridge::recv_loop() {
    uint8_t rbuf[2048];
    mavlink_message_t msg;
    mavlink_status_t status;
    sockaddr_in src{};
    socklen_t srclen = sizeof(src);

    while (running_) {
        int n = recvfrom(sock_, rbuf, sizeof(rbuf), 0, (struct sockaddr*)&src, &srclen);
        if (n <= 0) continue;
        for (int i=0; i<n; ++i) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, rbuf[i], &msg, &status)) continue;

            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                manager.discover(msg.sysid);

                std::lock_guard<std::mutex> lk(status_mtx);
                auto &ds = drone_status[msg.sysid];
                ds.sysid = msg.sysid;
                ds.last_hb = std::chrono::steady_clock::now();
                ds.has_addr = true; // no need to store src
            }
            else if (msg.msgid == MAVLINK_MSG_ID_SYS_STATUS) {
                mavlink_sys_status_t ss;
                mavlink_msg_sys_status_decode(&msg, &ss);
                update_sys_status(msg.sysid, ss.onboard_control_sensors_present);
                std::lock_guard<std::mutex> lk(status_mtx);
                auto &ds = drone_status[msg.sysid];
                ds.battery_percent = ss.battery_remaining;
            }
            else if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                mavlink_global_position_int_t gp;
                mavlink_msg_global_position_int_decode(&msg, &gp);
                update_position(msg.sysid, gp.lat, gp.lon, gp.relative_alt);
            }
            else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                mavlink_command_ack_t ack;
                mavlink_msg_command_ack_decode(&msg, &ack);

                std::lock_guard<std::mutex> lock(ack_mtx);
                auto key = std::make_pair(msg.sysid, ack.command);
                if (pending_acks.find(key) != pending_acks.end()) {
                    std::cout << "[ACK] Drone " << msg.sysid
                              << " Command " << ack.command
                              << " → " << ack_result_to_string(ack.result)
                              << std::endl;
                    pending_acks.erase(key);
                }
                update_ack(msg.sysid, ack.command, ack.result);
            }
        }
    }
}

// ---------------- Updates ----------------
void MavlinkBridge::update_sys_status(int sysid, uint8_t status) {
    std::lock_guard<std::mutex> lk(status_mtx);
    drone_status[sysid].system_status = status;
}

void MavlinkBridge::update_position(int sysid, int32_t lat_e7, int32_t lon_e7, int32_t alt_mm) {
    std::lock_guard<std::mutex> lk(status_mtx);
    auto &ds = drone_status[sysid];
    ds.lat_e7 = lat_e7;
    ds.lon_e7 = lon_e7;
    ds.alt_mm = alt_mm;
}

void MavlinkBridge::update_ack(int sysid, int command, int result) {
    {
        std::lock_guard<std::mutex> lk(status_mtx);
        auto &ds = drone_status[sysid];
        ds.last_command = command;
        ds.last_result = result;
    }
    std::unique_lock<std::mutex> lk(cv_mtx);
    last_ack_sysid = sysid;
    last_ack_command = command;
    last_ack_result = result;
    cv.notify_all();
}

