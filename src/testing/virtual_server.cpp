#include "virtual_server.h"
#include "../messages/json_parser.h"
#include "../core/timing_manager.h"

#include <iostream>
#include <nlohmann/json.hpp>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
#include <cmath>

// 🔹 MAVLink headers (adjust path to your local c_library_v2)
extern "C" {
#include "/home/zulu/libs/c_library_v2/common/mavlink.h"
}

struct Waypoint {
    double lat;
    double lon;
    double alt;
};

// 🔹 Shared queue for GoHere commands (listener still stores incoming GoHere if needed)
std::queue<Waypoint> waypoint_queue;
std::mutex waypoint_mutex;
std::condition_variable waypoint_cv;

VirtualC2Server::VirtualC2Server() {
    m_udp_handler = std::make_unique<UDPHandler>(
        VIRTUAL_SERVER_RECEIVE_PORT, "127.0.0.1", VIRTUAL_SERVER_SEND_PORT);
    m_aes_handler = std::make_unique<AESHandler>(AES_KEY, AES_IV);

    // --- TEST PLAN ---
    m_test_plan.push_back(ScenarioType::HAPPY_PATH);
    m_test_plan.push_back(ScenarioType::C2_ABORT_TEST);
}

std::string VirtualC2Server::get_current_scenario_name() const {
    if (m_current_scenario_index < m_test_plan.size()) {
        switch(m_test_plan[m_current_scenario_index]) {
            case ScenarioType::HAPPY_PATH: return "Happy Path Workflow";
            case ScenarioType::C2_ABORT_TEST: return "C2-Initiated Abort";
        }
    }
    return "Unknown";
}

void VirtualC2Server::send_encrypted_message(const std::string& json_payload) {
    std::string encrypted_msg = json_payload; // m_aes_handler->encrypt(json_payload);
    m_udp_handler->send(encrypted_msg);
}

// 🔹 MAVLink listener stores GoHere commands (unchanged)
void VirtualC2Server::start_mavlink_listener() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket");
        return;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(14550);   // QGC MAVLink port
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sock);
        return;
    }

    std::thread([sock]() {
        uint8_t buf[2048];
        mavlink_message_t msg;
        mavlink_status_t status;

        std::cout << "[MAVLINK] Listening for GoHere on UDP 14550..." << std::endl;

        while (true) {
            ssize_t n = recv(sock, buf, sizeof(buf), 0);
            if (n <= 0) continue;

            for (ssize_t i = 0; i < n; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                    if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
                        mavlink_command_long_t cmd;
                        mavlink_msg_command_long_decode(&msg, &cmd);

                        if (cmd.command == MAV_CMD_NAV_WAYPOINT || cmd.command == MAV_CMD_DO_REPOSITION) {
                            Waypoint wp{cmd.param5, cmd.param6, cmd.param7};
                            {
                                std::lock_guard<std::mutex> lock(waypoint_mutex);
                                waypoint_queue.push(wp);
                            }
                            waypoint_cv.notify_one();
                            std::cout << "[MAVLINK] Stored GoHere: "
                                      << wp.lat << ", " << wp.lon << ", " << wp.alt << std::endl;
                        }
                    } else if (msg.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT) {
                        mavlink_set_position_target_global_int_t sp;
                        mavlink_msg_set_position_target_global_int_decode(&msg, &sp);

                        Waypoint wp{sp.lat_int / 1e7, sp.lon_int / 1e7, sp.alt};
                        {
                            std::lock_guard<std::mutex> lock(waypoint_mutex);
                            waypoint_queue.push(wp);
                        }
                        waypoint_cv.notify_one();
                        std::cout << "[MAVLINK] Stored Guided GoTo: "
                                  << wp.lat << ", " << wp.lon << ", " << wp.alt << std::endl;
                    }
                }
            }
        }
    }).detach();
}

void VirtualC2Server::run_test_suite() {
    // Start MAVLink listener (keeps collecting GoHere commands if you still want that)
    start_mavlink_listener();

    if (!m_udp_handler->start([this](const std::string& msg) { this->on_message_received(msg); })) {
        std::cerr << "Failed to start Virtual C2 server." << std::endl;
        return;
    }

    const std::string DRONE_ID = "AA001";
    const std::string TRACK_ID = "JA101";

    // Generate session UUID
    m_session_uuid = JsonParser::generateUniqueId32();
    std::cout << "[C2] Session UUID = " << m_session_uuid << std::endl;

    // Step 1: Heartbeat
    std::cout << "\n[C2-ACTION] Sending C2 Heartbeat" << std::endl;
    send_encrypted_message(JsonParser::createC2Heartbeat("A001", m_session_uuid));
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // --- Circle simulation setup ---
    // Center coordinate (from your sample)
    constexpr double center_lat = -35.363096;
    constexpr double center_lon = 149.164760;
    constexpr double center_alt = 40.0; // meters
    double vel = 2.0;
    double vel1=7.0;
    // Circle parameters
    const double radius_deg = 0.0010; // ~111 m per 0.001° latitude (approx)
    const int points = 5;

    // Build 5 points evenly spaced on the circle
    std::vector<Waypoint> circle;
    circle.reserve(points);
    for (int i = 0; i < points; ++i) {
        double theta = 2.0 * M_PI * i / points;
        double lat = center_lat + radius_deg * std::cos(theta);
        double lon = center_lon + radius_deg * std::sin(theta);
        circle.push_back(Waypoint{lat, lon, center_alt});
        
    }

    // --- New requested behavior:
    // Send MissionAssignment ONCE (first circle point), wait 15s,
    // then send the 5 TPUs (one per circle point) at 10s intervals, each sent only once.
    // ---

    // Send MissionAssignment only once using the first circle point
    if (!circle.empty()) {
        const Waypoint &first_wp = circle[0];
        std::cout << "\n[C2-ACTION] Sending Mission Assignment (one-time) from circle first point: "
                  << first_wp.lat << ", " << first_wp.lon << ", " << first_wp.alt << std::endl;

        send_encrypted_message(JsonParser::createMissionAssignment(
            TRACK_ID, first_wp.lat, first_wp.lon, first_wp.alt, vel, m_session_uuid));
    } else {
        std::cerr << "[C2] Circle is empty! No MissionAssignment sent." << std::endl;
    }

    // Wait 15 seconds before sending TPUs
    std::this_thread::sleep_for(std::chrono::seconds(30));

    // Send 5 TPUs (one per circle point) at 10s intervals
    std::cout << "\n[C2-ACTION] Sending 5 TPUs (one-time each) at 10s intervals." << std::endl;
    for (int i = 0; i < points; ++i) {
        const Waypoint &wp = circle[i];
        std::cout << "\n[C2-ACTION] Sending Target Positional Update (" << (i+1)
                  << "/" << points << "): " << wp.lat << ", " << wp.lon << ", " << wp.alt << std::endl;

        send_encrypted_message(JsonParser::createTargetPositionalUpdate(
            TRACK_ID, DRONE_ID, wp.lat, wp.lon, wp.alt, vel1, m_session_uuid));

        // Wait 10 seconds before the next TPU (but don't wait after the last one)
        if (i < points - 1) {
            std::this_thread::sleep_for(std::chrono::seconds(20));
        }
    }

    std::cout << "\n[C2-ACTION] Completed sending 5 TPUs." << std::endl;

    // --- Continue the rest of your scenario ---
    std::cout << "\n[C2-ACTION] Sending Chase Mode Ack (1604)" << std::endl;
    send_encrypted_message(JsonParser::createChaseModeAck(m_session_uuid, DRONE_ID, TRACK_ID, 1));
    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::cout << "\n[C2-ACTION] Sending Mission Completion Ack (1605)" << std::endl;
    send_encrypted_message(JsonParser::createMissionCompletionAck(TRACK_ID, DRONE_ID, m_session_uuid));
    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::cout << "\n[C2-ACTION] Sending Mission Abort Command (1606)" << std::endl;
    send_encrypted_message(JsonParser::createMissionAbortC2Cmd(TRACK_ID, DRONE_ID, m_session_uuid));

    std::cout << "\n--- All Test Scenarios Sent ---\n" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(2));
    m_udp_handler->stop();
}

void VirtualC2Server::on_message_received(const std::string& encrypted_b64_message) {
    std::string decrypted_message = encrypted_b64_message; // m_aes_handler->decrypt(encrypted_b64_message);
    if (decrypted_message.empty()) return;

    std::cout << "[C2] Received From Hub: " << decrypted_message << std::endl;
    m_last_request_id = JsonParser::getUniqueId(decrypted_message);

    int msg_id = JsonParser::getMessageId(decrypted_message);
    if (msg_id == static_cast<int>(MessageId::MISSION_COMPLETED)) {
        std::cout << "[C2-REACTION] Hub reported mission completion. Preparing to acknowledge."
                  << std::endl;
        std::lock_guard<std::mutex> lock(m_state_mutex);
        m_current_test_state = TestState::WAITING_FOR_COMPLETION_ACK;
    } else if (msg_id == static_cast<int>(MessageId::ACK_MISSION_ABORT_C2)) {
        std::cout << "[C2-REACTION] Hub correctly acknowledged the C2 Abort command." << std::endl;
    }
}

