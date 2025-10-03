#include "drone_hub_server.h"
#include "../messages/json_parser.h"
#include "../core/timing_manager.h"
#include "../communication/cannister.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <random>

using json = nlohmann::json;

DroneHubServer::DroneHubServer(MavlinkBridge& mavlink_bridge)
    : m_mavlink_bridge(mavlink_bridge) {
    m_udp_handler = std::make_unique<UDPHandler>(
        DRONE_HUB_RECEIVE_PORT, "192.168.194.153", DRONE_HUB_SEND_PORT);
    m_aes_handler = std::make_unique<AESHandler>(AES_KEY, AES_IV);

    load_track_config("track_config.json");

    m_session_uuid = JsonParser::generateUniqueId32();
    std::cout << "[HUB] Session UUID = " << m_session_uuid << std::endl;
    rtspManager = std::make_unique<RTSPManager>(8554);
}

void DroneHubServer::send_encrypted_message(const std::string& json_payload) {
    std::string msg = json_payload; // no encryption for now
    m_udp_handler->send(msg);
}

void DroneHubServer::run() {
    if (!m_udp_handler->start(
            [this](const std::string& msg) { this->on_message_received(msg); })) {
        std::cerr << "Failed to start Drone Hub server." << std::endl;
        return;
    }
    
    try {
        rtspManager->loadConfigYAML("inputs.yaml");  // use config
    } catch (...) {
        std::cout << "[RTSP] No config → auto assigning devices" << std::endl;
        rtspManager->autoAssignDevices();        // fallback
    }

    TimingManager timer(1.0);
    while (true) {
        send_status_update();
        std::cout << "[HUB] Current State: "
                  << m_state_machine.state_to_string(
                         m_state_machine.get_current_state())
                  << std::endl;
        timer.wait_for_next_tick();
    }
}

void DroneHubServer::on_message_received(const std::string& message) {
    std::string decrypted = message; // no encryption
    if (decrypted.empty()) return;

    std::cout << "[HUB] Received Decrypted: " << decrypted << std::endl;
    process_message(decrypted);
}

void DroneHubServer::process_message(const std::string& json_message) {
    try {
        json parsed = json::parse(json_message);
        auto msg_id = static_cast<MessageId>(parsed.at("message_id").get<int>());
        std::string unique_id = JsonParser::getUniqueId(json_message);

        MissionState current_state = m_state_machine.get_current_state();

        // Extract track ID if present
        std::string track_id;
        if (parsed.contains("message_text") &&
            parsed["message_text"].contains("track_id")) {
            track_id = parsed["message_text"]["track_id"].get<std::string>();
        }

        // Resolve assigned drone
        std::string assigned_drone = get_drone_from_track(track_id);

        if (!track_id.empty() && assigned_drone == "UNKNOWN") {
            std::cerr << "[HUB] Track " << track_id
                      << " not mapped to any drone. Ignoring.\n";
            return;
        }

        switch (msg_id) {
            case MessageId::C2_HEARTBEAT:
                break;

            case MessageId::MISSION_ASSIGNMENT: {
                if (current_state == MissionState::IDLE && assigned_drone != "UNKNOWN") {
                    m_state_machine.set_state(MissionState::LAUNCHED);

                    // Activate heartbeat for this drone
                    active_missions[track_id] = assigned_drone;

                    send_encrypted_message(JsonParser::createMissionAck(
                        unique_id, assigned_drone, track_id, 1));

                    const auto& msg_text = parsed.at("message_text");
                    double lat = msg_text.at("target_lat").get<double>();
                    double lon = msg_text.at("target_long").get<double>();
                    double alt = msg_text.at("target_alt").get<double>();
                    double v   = msg_text.at("target_vel").get<double>();

                    std::cout << "\n[HUB-ACTION] Relaying Mission to Drone "
                              << assigned_drone << "...\n";
                    auto drones = m_mavlink_bridge.snapshot_status();
                    for (auto &kv : drones) {
                        if (m_mavlink_bridge.get_manager().get_cdrone_id(kv.first) ==
                            assigned_drone) {
                            int sysid = kv.first;
                            sendCommandsToSerial("/dev/ttyUSB1", "2\n");
                            std::cout << "sent launch cmd" << std::endl;
                            std::this_thread::sleep_for(std::chrono::seconds(30));
                            //m_mavlink_bridge.arm(sysid);
                            //m_mavlink_bridge.takeoff(sysid, 10.0f);
                            m_mavlink_bridge.set_mode_guided(sysid);
                            
                            //std::this_thread::sleep_for(std::chrono::seconds(30));
                            m_mavlink_bridge.reposition(sysid, lat, lon, alt, v);
                        }
                    }
                } else {
                    send_encrypted_message(JsonParser::createMissionAck(
                        unique_id, assigned_drone, track_id, -1));
                }
                break;
            }

            case MessageId::TARGET_POSITIONAL_UPDATE: {
                if (assigned_drone != "UNKNOWN" &&
                    (current_state == MissionState::LAUNCHED ||
                     current_state == MissionState::MOVE_TO_TARGET)) {
                    m_state_machine.set_state(MissionState::MOVE_TO_TARGET);

                    const auto& msg_text = parsed.at("message_text"); 
                    double lat = msg_text.at("target_lat").get<double>();
                    double lon = msg_text.at("target_long").get<double>();
                    double alt = msg_text.at("target_alt").get<double>();
                    double v   = msg_text.at("target_vel").get<double>();

                    auto drones = m_mavlink_bridge.snapshot_status();
                    for (auto &kv : drones) {
                        if (m_mavlink_bridge.get_manager().get_cdrone_id(kv.first) ==
                            assigned_drone) {
                            int sysid = kv.first;
                            m_mavlink_bridge.reposition(sysid, lat, lon, alt, v);
                            
                        }
                    }
                }
                break;
            }

            case MessageId::CHASE_MODE_ACK: {
                 
                if (assigned_drone != "UNKNOWN") {
                    send_encrypted_message(JsonParser::createChaseModeActivated(
                        assigned_drone, track_id,unique_id ));
                    m_state_machine.set_state(MissionState::CHASE_MODE);
                }
                break;
            }

            case MessageId::MISSION_COMPLETION_ACK: {
                if (assigned_drone != "UNKNOWN") {
                    send_encrypted_message(JsonParser::createMissionCompleted(
                        track_id, assigned_drone, m_session_uuid));

                    // Deactivate this mission → stop sending HB
                    active_missions.erase(track_id);

                    m_state_machine.set_state(MissionState::IDLE);
                }
                break;
            }

            case MessageId::MISSION_ABORT_C2_CMD: {
                if (assigned_drone != "UNKNOWN") {
                    std::cout << "[HUB-ACTION] Mission Abort Received for "
                              << assigned_drone << ". Sending RTL...\n";
                    auto drones = m_mavlink_bridge.snapshot_status();
                    for (auto &kv : drones) {
                        if (m_mavlink_bridge.get_manager().get_cdrone_id(kv.first) ==
                            assigned_drone) {
                            int sysid = kv.first;
                            m_mavlink_bridge.rtl(sysid);
                        }
                    }
                    send_encrypted_message(
                        JsonParser::createAckMissionAbortFromC2(
                            track_id, assigned_drone, unique_id));

                    // Deactivate this mission → stop sending HB
                    active_missions.erase(track_id);

                    m_state_machine.set_state(MissionState::IDLE);
                }
                break;
            }

            default:
                std::cerr << "[HUB] Unknown message ID: " << (int)msg_id
                          << std::endl;
                break;
        }
    } catch (const std::exception &e) {
        std::cerr << "[HUB] Error processing message: " << e.what() << std::endl;
    }
}

void DroneHubServer::send_status_update() {
    // Always send hub status
    // Collect live drone status from bridge
    auto drones = m_mavlink_bridge.snapshot_status();

    // --- Battery status array ---
    std::vector<int> battery_array;
    for (auto &kv : drones) {
        const DroneStatus &ds = kv.second;
        int batt = (ds.battery_percent >= 0 && ds.battery_percent <= 100)
                       ? ds.battery_percent
                       : 100;  // fallback
        battery_array.push_back(batt);
    }

    // --- Counts ---
    int cdrone_total = static_cast<int>(drones.size());
    int cdrone_readiness = cdrone_total; // TODO: real readiness check
    int cdrone_docked = 0;               // TODO: real docked check

    // --- Build hub status message ---
    std::string hub_status = JsonParser::createDroneHubStatus(
        "A001",            // hub ID
        cdrone_docked,
        cdrone_readiness,
        cdrone_total,
        battery_array,     // dynamic battery list
        m_session_uuid
    );

    send_encrypted_message(hub_status);
    
    for (const auto &entry : active_missions) {
        const std::string &track_id  = entry.first;
        const std::string &cdrone_id = entry.second;

        for (auto &kv : drones) {
            std::string cid = m_mavlink_bridge.get_manager().get_cdrone_id(kv.first);
            if (cid == cdrone_id) {
                DroneStatus ds = kv.second;

                int battery = 100;
                if (ds.system_status > 0) {
                    battery = 95;
                }

                auto hb_msg = JsonParser::createDroneHeartbeat(
                    cdrone_id, track_id,
                    ds.hb_status,
                    ds.battery_percent,
                    ds.weapon_readiness,
                    ds.weapon_engaged,
                    ds.lat_e7 / 1e7, ds.lon_e7 / 1e7, ds.alt_mm / 1000.0,
                    0.0,
                    ds.last_command, ds.last_result,
                    m_session_uuid);

                send_encrypted_message(hb_msg);
            }
        }
    }
}

void DroneHubServer::load_track_config(const std::string &filename) {
    std::ifstream f(filename);
    if (!f.is_open()) {
        std::cerr << "[HUB] Track config file not found: " << filename << std::endl;
        return;
    }
    try {
        json cfg;
        f >> cfg;
        for (auto it = cfg.begin(); it != cfg.end(); ++it) {
            drone_track_map[it.key()] = it.value();
            std::cout << "[HUB] Drone " << it.key()
                      << " mapped to track " << it.value() << std::endl;
        }
    } catch (const std::exception &e) {
        std::cerr << "[HUB] Error parsing track config: " << e.what() << std::endl;
    }
}

std::string DroneHubServer::get_drone_from_track(const std::string &track_id) {
    for (const auto &p : drone_track_map) {
        if (p.second == track_id) return p.first;
    }
    return "UNKNOWN";
}

