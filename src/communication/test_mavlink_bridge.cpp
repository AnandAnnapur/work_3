#include <iostream>
#include <thread>
#include <chrono>
#include <atomic>
#include <map>
#include <mutex>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

// MAVLink headers (adjust path)
extern "C" {
#include "/home/zulu/libs/c_library_v2/common/mavlink.h"
}

int sockfd = -1;
sockaddr_in router_addr{};
std::atomic<bool> running{false};

// discovered drones: sysid -> compid
std::map<int,int> discovered;
std::mutex disc_mtx;

// store ACKs: command -> result
std::map<int,int> last_ack;
std::mutex ack_mtx;

// ---------- msgid lookup ----------
std::string msg_name(int id) {
    switch (id) {
        case MAVLINK_MSG_ID_HEARTBEAT: return "HEARTBEAT";
        case MAVLINK_MSG_ID_SYS_STATUS: return "SYS_STATUS";
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: return "GLOBAL_POSITION_INT";
        case MAVLINK_MSG_ID_COMMAND_ACK: return "COMMAND_ACK";
        default: return "msgid=" + std::to_string(id);
    }
}

// ---------- send helper ----------
void send_mavlink(const mavlink_message_t &msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int len = mavlink_msg_to_send_buffer(buf, &msg);
    sendto(sockfd, buf, len, 0, (struct sockaddr*)&router_addr, sizeof(router_addr));
}

// ---------- commands ----------
void set_mode_guided(int sysid, int compid) {
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(
        1, MAV_COMP_ID_MISSIONPLANNER, &msg,
        sysid,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4 // GUIDED
    );
    send_mavlink(msg);
    std::cout << "[SEND] SET_MODE GUIDED -> sysid=" << sysid << " compid=" << compid << "\n";
}

void send_arm(int sysid, int compid) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        1, MAV_COMP_ID_MISSIONPLANNER, &msg,
        sysid, compid,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0,0,0,0,0,0
    );
    send_mavlink(msg);
    std::cout << "[SEND] ARM -> sysid=" << sysid << " compid=" << compid << "\n";
}

void send_takeoff(int sysid, int compid, float alt) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        1, MAV_COMP_ID_MISSIONPLANNER, &msg,
        sysid, compid,
        MAV_CMD_NAV_TAKEOFF,
        0,0,0,0,0,0,0, alt
    );
    send_mavlink(msg);
    std::cout << "[SEND] TAKEOFF " << alt << "m -> sysid=" << sysid << " compid=" << compid << "\n";
}

void send_rtl(int sysid, int compid) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        1, MAV_COMP_ID_MISSIONPLANNER, &msg,
        sysid, compid,
        MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,0,0,0,0,0,0,0
    );
    send_mavlink(msg);
    std::cout << "[SEND] RTL -> sysid=" << sysid << " compid=" << compid << "\n";
}

// ---------- request telemetry ----------
void request_data_stream(int sysid, int compid) {
    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(
        1, MAV_COMP_ID_MISSIONPLANNER, &msg,
        sysid, compid,
        MAV_DATA_STREAM_ALL,  // request all data
        10,                   // 10 Hz
        1                     // start
    );
    send_mavlink(msg);
    std::cout << "[SEND] REQUEST_DATA_STREAM -> sysid=" << sysid << " compid=" << compid << "\n";
}

// ---------- GCS heartbeat ----------
void send_gcs_heartbeat() {
    mavlink_message_t hb;
    mavlink_msg_heartbeat_pack(
        1, MAV_COMP_ID_MISSIONPLANNER, &hb,
        MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0,0,0
    );
    send_mavlink(hb);
}

void hb_loop() {
    while (running.load()) {
        send_gcs_heartbeat();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// ---------- Receiver ----------
void recv_loop() {
    uint8_t buf[2048];
    mavlink_message_t msg;
    mavlink_status_t status;

    while (running.load()) {
        sockaddr_in src{};
        socklen_t srclen = sizeof(src);
        ssize_t n = recvfrom(sockfd, buf, sizeof(buf), 0, (struct sockaddr*)&src, &srclen);
        if (n < 0) continue;

        for (ssize_t i = 0; i < n; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                std::cout << "[RECV] " << msg_name(msg.msgid)
                          << " sysid=" << (int)msg.sysid
                          << " compid=" << (int)msg.compid << std::endl;

                if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&msg, &hb);
                    if (hb.autopilot != MAV_AUTOPILOT_INVALID &&
                        hb.type != MAV_TYPE_GCS) {
                        std::lock_guard<std::mutex> lk(disc_mtx);
                        discovered[msg.sysid] = msg.compid;
                        std::cout << "[DISCOVER] sysid=" << (int)msg.sysid
                                  << " compid=" << (int)msg.compid
                                  << " type=" << (int)hb.type << std::endl;
                    }
                } else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                    mavlink_command_ack_t ack;
                    mavlink_msg_command_ack_decode(&msg, &ack);
                    {
                        std::lock_guard<std::mutex> lk(ack_mtx);
                        last_ack[ack.command] = ack.result;
                    }
                    std::cout << "[ACK] sysid=" << (int)msg.sysid
                              << " command=" << ack.command
                              << " result=" << (int)ack.result << "\n";
                }
            }
        }
    }
}

// ---------- Wait for ACK ----------
int wait_for_ack(int command, int timeout_sec = 5) {
    for (int i = 0; i < timeout_sec * 10; ++i) {
        {
            std::lock_guard<std::mutex> lk(ack_mtx);
            auto it = last_ack.find(command);
            if (it != last_ack.end()) return it->second;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return -1;
}

// ---------- main ----------
int main() {
    const char* router_ip = "127.0.0.1";
    const int router_port = 14600;

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) { perror("socket"); return 1; }

    // bind to ephemeral port
    sockaddr_in local{};
    memset(&local, 0, sizeof(local));
    local.sin_family = AF_INET;
    local.sin_port = htons(0); // OS chooses port
    local.sin_addr.s_addr = INADDR_ANY;

    if (bind(sockfd, (struct sockaddr*)&local, sizeof(local)) < 0) {
        perror("bind");
        close(sockfd);
        return 1;
    }

    // router address
    memset(&router_addr, 0, sizeof(router_addr));
    router_addr.sin_family = AF_INET;
    router_addr.sin_port = htons(router_port);
    inet_aton(router_ip, &router_addr.sin_addr);

    std::cout << "[INFO] Client started, sending to " << router_ip << ":" << router_port << "\n";

    running.store(true);
    std::thread rthr(recv_loop);
    std::thread hbthr(hb_loop);

    // wait for discovery
    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::map<int,int> snapshot;
    {
        std::lock_guard<std::mutex> lk(disc_mtx);
        snapshot = discovered;
    }

    if (snapshot.empty()) {
        std::cerr << "[ERROR] No drones discovered\n";
    } else {
        for (auto &kv : snapshot) {
            int sysid = kv.first;
            int compid = kv.second;

            std::cout << "[INFO] Working with sysid=" << sysid << " compid=" << compid << "\n";

            request_data_stream(sysid, compid);

            set_mode_guided(sysid, compid);
            int res = wait_for_ack(MAV_CMD_DO_SET_MODE);
            std::cout << " -> SET_MODE result=" << res << "\n";

            send_arm(sysid, compid);
            res = wait_for_ack(MAV_CMD_COMPONENT_ARM_DISARM);
            std::cout << " -> ARM result=" << res << "\n";

            send_takeoff(sysid, compid, 10.0f);
            res = wait_for_ack(MAV_CMD_NAV_TAKEOFF);
            std::cout << " -> TAKEOFF result=" << res << "\n";

            std::this_thread::sleep_for(std::chrono::seconds(5));

            send_rtl(sysid, compid);
            res = wait_for_ack(MAV_CMD_NAV_RETURN_TO_LAUNCH);
            std::cout << " -> RTL result=" << res << "\n";
        }
    }

    running.store(false);
    rthr.join();
    hbthr.join();
    close(sockfd);
    return 0;
}

