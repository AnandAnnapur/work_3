#pragma once
#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include "../communication/udp_handler.h"
#include "../encryption/aes_handler.h"

enum class ScenarioType {
    HAPPY_PATH,
    C2_ABORT_TEST
};

class VirtualC2Server {
public:
    VirtualC2Server();

    std::string get_current_scenario_name() const;
    void send_encrypted_message(const std::string& json_payload);
    void run_test_suite();
    void on_message_received(const std::string& encrypted_b64_message);

    // 🔹 Add this line
    void start_mavlink_listener();

private:
    std::unique_ptr<UDPHandler> m_udp_handler;
    std::unique_ptr<AESHandler> m_aes_handler;
    std::string m_session_uuid;
    std::string m_last_request_id;

    std::vector<ScenarioType> m_test_plan;
    size_t m_current_scenario_index = 0;

    enum class TestState {
        IDLE,
        WAITING_FOR_COMPLETION_ACK
    };
    TestState m_current_test_state = TestState::IDLE;
    mutable std::mutex m_state_mutex;
};

