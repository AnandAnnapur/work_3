#include "json_parser.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <random>

// --- Utility Functions ---

std::string JsonParser::generateUniqueId32() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<uint32_t> distrib(0, 0xFFFFFFFF);

    auto genHex = [&](int width) {
        std::stringstream ss;
        ss << std::hex << std::setw(width) << std::setfill('0') << distrib(gen);
        return ss.str();
    };

    // UUID v4 format: 8-4-4-4-12
    std::stringstream uuid;
    uuid << genHex(8) << "-"
         << genHex(4) << "-"
         << "4" << genHex(3) << "-"                // version 4
         << (8 + distrib(gen) % 4) << genHex(3) << "-" // variant 10xx
         << genHex(12);

    return uuid.str();
}

std::string JsonParser::getCurrentTimestamp() {
    const auto now = std::chrono::system_clock::now();
    const auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::gmtime(&time_t_now), "%Y-%m-%dT%H:%M:%SZ");
    return ss.str();
}

int JsonParser::getMessageId(const std::string& json_str) {
    try {
        json j = json::parse(json_str);
        if (j.contains("message_id")) {
            return j["message_id"].get<int>();
        }
    } catch (...) {}
    return -1;
}

std::string JsonParser::getUniqueId(const std::string& json_str) {
    try {
        json j = json::parse(json_str);
        if (j.contains("message_unique_id")) {
            return j["message_unique_id"].get<std::string>();
        }
    } catch (...) {}
    return "";
}

// --- C2 -> DroneHub ---

std::string JsonParser::createC2Heartbeat(const std::string& hub_id, std::string session_uuid) {
    json j;
    j["message_id"] = static_cast<int>(MessageId::C2_HEARTBEAT);
    j["message_type"] = "HB";
    j["message_unique_id"] = session_uuid;
    j["message_text"] = {
        {"dronehub_id", hub_id},
        {"hb_status", 1},
        {"timestamp", getCurrentTimestamp()}
    };
    return j.dump();
}

std::string JsonParser::createMissionAssignment(const std::string& track_id,
                                                double lat, double lon, double alt, double vel,
                                                std::string session_uuid) {
    json j;
    j["message_id"] = static_cast<int>(MessageId::MISSION_ASSIGNMENT);
    j["message_type"] = "MT";
    j["message_unique_id"] = session_uuid;
    j["message_text"] = {
        {"track_id", track_id},
        {"target_lat", lat},
        {"target_long", lon},
        {"target_alt", alt},
        {"target_vel", vel},
        {"timestamp", getCurrentTimestamp()}
    };
    return j.dump();
}

std::string JsonParser::createTargetPositionalUpdate(const std::string& track_id,
                                                     const std::string& drone_id,
                                                     double lat, double lon, double alt, double vel,
                                                     std::string session_uuid) {
    json j;
    j["message_id"] = static_cast<int>(MessageId::TARGET_POSITIONAL_UPDATE);
    j["message_type"] = "TPU";
    j["message_unique_id"] = session_uuid;
    j["message_text"] = {
        {"track_id", track_id},
        {"cdrone_id", drone_id},
        {"target_lat", lat},
        {"target_long", lon},
        {"target_alt", alt},
        {"target_vel", vel},
        {"timestamp", getCurrentTimestamp()}
    };
    return j.dump();
}

std::string JsonParser::createChaseModeAck(std::string unique_id,
                                           const std::string& drone_id,
                                           const std::string& track_id,
                                           int ack) {
    json j;
    j["message_id"] = static_cast<int>(MessageId::CHASE_MODE_ACK);
    j["message_type"] = "CMA";
    j["message_unique_id"] = unique_id;
    j["message_text"] = {
        {"cdrone_id", drone_id},
        {"track_id", track_id},
        {"ack", ack}
    };
    return j.dump();
}

std::string JsonParser::createMissionCompletionAck(const std::string& track_id,
                                                   const std::string& drone_id,
                                                   std::string session_uuid) {
    json j;
    j["message_id"] = static_cast<int>(MessageId::MISSION_COMPLETION_ACK);
    j["message_type"] = "MC";
    j["message_unique_id"] = session_uuid;
    j["message_text"] = {
        {"track_id", track_id},
        {"cdrone_id", drone_id},
        {"timestamp", getCurrentTimestamp()}
    };
    return j.dump();
}

std::string JsonParser::createMissionAbortC2Cmd(const std::string& track_id,
                                                const std::string& drone_id,
                                                std::string session_uuid) {
    json j;
    j["message_id"] = static_cast<int>(MessageId::MISSION_ABORT_C2_CMD);
    j["message_type"] = "MAC";
    j["message_unique_id"] = session_uuid;
    j["message_text"] = {
        {"track_id", track_id},
        {"cdrone_id", drone_id},
        {"timestamp", getCurrentTimestamp()}
    };
    return j.dump();
}

std::string JsonParser::createAckForMissionAbortFromDH(const std::string& track_id,
                                                       const std::string& drone_id,
                                                       std::string session_uuid) {
    json j;
    j["message_id"] = static_cast<int>(MessageId::ACK_MISSION_ABORT_DH);
    j["message_type"] = "MAD";
    j["message_unique_id"] = session_uuid;
    j["message_text"] = {
        {"track_id", track_id},
        {"cdrone_id", drone_id},
        {"timestamp", getCurrentTimestamp()}
    };
    return j.dump();
}

// --- DroneHub -> C2 ---

std::string JsonParser::createDroneHubStatus(const std::string& hub_id,
                                             int total, int docked, int ready,
                                             const std::vector<int>& battery_status,
                                             std::string session_uuid) {
    json j;
    j["message_id"] = static_cast<int>(MessageId::DRONEHUB_HEARTBEAT_AND_STATUS);
    j["message_type"] = "HB";
    j["message_unique_id"] = session_uuid;
    j["message_text"] = {
        {"dronehub_id", hub_id},
        {"hb_status", 1},
        {"cdrone_total", total},
        {"cdrone_docked", docked},
        {"cdrone_readiness", ready},
        {"battery_status", battery_status},
        {"time", getCurrentTimestamp()}
    };
    return j.dump();
}

std::string JsonParser::createDroneHeartbeat(const std::string& drone_id,
                                             const std::string& track_id,
                                             int hb_status,
                                             int battery_status,
                                             int weapon_readiness,
                                             int weapon_engaged,
                                             double lat,
                                             double lon,
                                             double alt,
                                             double vel,
                                             int last_command,
                                             int last_result,
                                             std::string session_uuid) {
    json j;
    j["message_id"] = static_cast<int>(MessageId::DRONE_HEARTBEAT);
    j["message_type"] = "HB";
    j["message_unique_id"] = session_uuid;
    j["message_text"] = {
        {"cdrone_id", drone_id},
        {"hb_status", hb_status},
        {"battery_status", battery_status},
        {"weapon_readiness", weapon_readiness},
        {"weapon_engaged", weapon_engaged},
        {"track_id", track_id},
        {"cdrone_lat", lat},
        {"cdrone_long", lon},
        {"cdrone_alt", alt},
        {"cdrone_vel", vel},
        {"last_command", last_command},
        {"last_result", last_result},
        {"timestamp", getCurrentTimestamp()}
    };
    return j.dump();
}

std::string JsonParser::createMissionAck(std::string unique_id_from_req,
                                         const std::string& drone_id,
                                         const std::string& track_id,
                                         int ack_status) {
    json j;
    j["message_id"] = static_cast<int>(MessageId::MISSION_ACKNOWLEDGEMENT);
    j["message_type"] = "MT";
    j["message_unique_id"] = unique_id_from_req;
    j["message_text"] = {
        {"cdrone_id", drone_id},
        {"track_id", track_id},
        {"msg_ack", std::to_string(ack_status)},
        {"timestamp", getCurrentTimestamp()}
    };
    return j.dump();
}

std::string JsonParser::createChaseModeActivated(const std::string& drone_id,
                                                 const std::string& track_id,
                                                 std::string session_uuid) {
    json j;
    j["message_id"] = static_cast<int>(MessageId::CHASE_MODE_ACTIVATED);
    j["message_type"] = "CMA";
    j["message_unique_id"] = session_uuid;
    j["message_text"] = {
        {"track_id", track_id},
        {"cdrone_id", drone_id},
        {"timestamp", getCurrentTimestamp()}
    };
    return j.dump();
}

std::string JsonParser::createMissionCompleted(const std::string& drone_id,
                                               const std::string& track_id,
                                               std::string session_uuid) {
    json j;
    j["message_id"] = static_cast<int>(MessageId::MISSION_COMPLETED);
    j["message_type"] = "MC";
    j["message_unique_id"] = session_uuid;
    j["message_text"] = {
        {"track_id", track_id},
        {"cdrone_id", drone_id},
        {"timestamp", getCurrentTimestamp()}
    };
    return j.dump();
}

std::string JsonParser::createAckMissionAbortFromC2(std::string unique_id_from_req,
                                                    const std::string& drone_id,
                                                    const std::string& track_id) {
    json j;
    j["message_id"] = static_cast<int>(MessageId::ACK_MISSION_ABORT_C2);
    j["message_type"] = "MAC";
    j["message_unique_id"] = unique_id_from_req;
    j["message_text"] = {
        {"track_id", track_id},
        {"cdrone_id", drone_id},
        {"timestamp", getCurrentTimestamp()}
    };
    return j.dump();
}

std::string JsonParser::createMissionAbortFromDH(const std::string& track_id,
                                                 const std::string& drone_id,
                                                 std::string session_uuid) {
    json j;
    j["message_id"] = static_cast<int>(MessageId::MISSION_ABORT_DH);
    j["message_type"] = "MAD";
    j["message_unique_id"] = session_uuid;
    j["message_text"] = {
        {"track_id", track_id},
        {"cdrone_id", drone_id},
        {"timestamp", getCurrentTimestamp()}
    };
    return j.dump();
}

