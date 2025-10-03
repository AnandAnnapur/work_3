#include "virtual_server.h"
#include "../messages/json_parser.h"
#include "../core/timing_manager.h"

#include <iostream>
#include <nlohmann/json.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
#include <cmath>

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
        switch (m_test_plan[m_current_scenario_index]) {
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

void VirtualC2Server::run_test_suite() {
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

    // --- Fixed 3-point mission setup ---
    const double alt_m = 40.0;
    struct MissionPoint { double lat; double lon; double alt; double vel; };
    std::vector<MissionPoint> mission_points = {
        {13.0537952, 77.6743949, alt_m, 3.0},  // Point 1
        {13.0543077, 77.6742931, alt_m, 10.0}, // Point 2
        {13.0544364, 77.6745946, alt_m, 8.0}   // Point 3
    };

    const int points = mission_points.size();
    const double min_total_seconds = 60.0;

    // --- MissionAssignment ---
    if (!mission_points.empty()) {
        const MissionPoint &first_wp = mission_points[0];
        std::cout << "\n[C2-ACTION] Sending Mission Assignment (first point): "
                  << first_wp.lat << ", " << first_wp.lon << ", " << first_wp.alt
                  << " @ vel=" << first_wp.vel << " m/s" << std::endl;

        send_encrypted_message(JsonParser::createMissionAssignment(
            TRACK_ID, first_wp.lat, first_wp.lon, first_wp.alt, points, m_session_uuid));
    }

    std::this_thread::sleep_for(std::chrono::seconds(60));

    int interval_sec = static_cast<int>(std::ceil(min_total_seconds / static_cast<double>(points)));
    if (interval_sec < 1) interval_sec = 1;
    std::cout << "[C2] TPU interval (sec) = " << interval_sec << std::endl;

    // --- TPUs ---
    std::cout << "\n[C2-ACTION] Sending " << points << " TPUs (fixed mission points)." << std::endl;
    for (int i = 0; i < points; ++i) {
        const MissionPoint &wp = mission_points[i];
        std::cout << "\n[C2-ACTION] Sending TPU (" << (i+1) << "/" << points
                  << "): " << wp.lat << ", " << wp.lon << ", " << wp.alt
                  << " @ vel=" << wp.vel << " m/s" << std::endl;

        send_encrypted_message(JsonParser::createTargetPositionalUpdate(
            TRACK_ID, DRONE_ID, wp.lat, wp.lon, wp.alt, wp.vel, m_session_uuid));

        if (i < points - 1) {
            std::this_thread::sleep_for(std::chrono::seconds(interval_sec));
        }
    }

    std::cout << "\n[C2-ACTION] Completed sending " << points << " TPUs." << std::endl;

    // --- Wrap up ---
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

