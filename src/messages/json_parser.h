#pragma once
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "message_types.h"

using json = nlohmann::json;

class JsonParser {
public:
    // Utilities
    static std::string generateUniqueId32();
    static std::string getCurrentTimestamp();
    static int getMessageId(const std::string& json_str);
    static std::string getUniqueId(const std::string& json_str);

    // --- C2 -> DroneHub ---
    static std::string createC2Heartbeat(const std::string& hub_id, std::string session_uuid);

    static std::string createMissionAssignment(const std::string& track_id,
                                               double lat, double lon, double alt, double vel,
                                               std::string session_uuid);

    static std::string createTargetPositionalUpdate(const std::string& track_id,
                                                    const std::string& drone_id,
                                                    double lat, double lon, double alt, double vel,
                                                    std::string session_uuid);

    static std::string createChaseModeAck(std::string unique_id,
                                          const std::string& drone_id,
                                          const std::string& track_id,
                                          int ack);

    static std::string createMissionCompletionAck(const std::string& track_id,
                                                  const std::string& drone_id,
                                                  std::string session_uuid);

    static std::string createMissionAbortC2Cmd(const std::string& track_id,
                                               const std::string& drone_id,
                                               std::string session_uuid);

    static std::string createAckForMissionAbortFromDH(const std::string& track_id,
                                                      const std::string& drone_id,
                                                      std::string session_uuid);

    // --- DroneHub -> C2 ---
    static std::string createDroneHubStatus(const std::string& hub_id,
                                            int total, int docked, int ready,
                                            const std::vector<int>& battery_status,
                                            std::string session_uuid);

    static std::string createDroneHeartbeat(const std::string& drone_id,
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
                                            std::string session_uuid);

    static std::string createMissionAck(std::string unique_id_from_req,
                                        const std::string& drone_id,
                                        const std::string& track_id,
                                        int ack_status);

    static std::string createChaseModeActivated(const std::string& drone_id,
                                                const std::string& track_id,
                                                std::string session_uuid);

    static std::string createMissionCompleted(const std::string& drone_id,
                                              const std::string& track_id,
                                              std::string session_uuid);

    static std::string createAckMissionAbortFromC2(std::string unique_id_from_req,
                                                   const std::string& drone_id,
                                                   const std::string& track_id);

    static std::string createMissionAbortFromDH(const std::string& track_id,
                                                const std::string& drone_id,
                                                std::string session_uuid);
};

