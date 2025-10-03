#pragma once

#include "udp_handler.h"
#include "../encryption/aes_handler.h"
#include "mavlink_bridge.h"
#include "../core/state_machine.h"
#include "../messages/json_parser.h"
#include "rtp.h"

#include <memory>
#include <unordered_map>
#include <string>
#include <nlohmann/json.hpp>

class DroneHubServer {
public:
    DroneHubServer(MavlinkBridge& mavlink_bridge);

    void run();
    void on_message_received(const std::string& message);
    void process_message(const std::string& json_message);
    void send_status_update();

private:
    void send_encrypted_message(const std::string& json_payload);
    void load_track_config(const std::string &filename);
    std::string get_drone_from_track(const std::string &track_id);

    // References
    MavlinkBridge& m_mavlink_bridge;

    // Communication
    std::unique_ptr<UDPHandler> m_udp_handler;
    std::unique_ptr<AESHandler> m_aes_handler;

    // State
    StateMachine m_state_machine;
    std::string m_session_uuid;

    // Config mappings
    std::unordered_map<std::string, std::string> drone_track_map;   // cdrone_id -> track_id
    std::unordered_map<std::string, std::string> active_missions;   // track_id -> cdrone_id
    
    //rtsp steam
    std::unique_ptr<RTSPManager> rtspManager;
};

