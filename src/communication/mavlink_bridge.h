#pragma once
#include <map>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <netinet/in.h>
#include "communication/drone_manager.h"

extern "C" {
#include "common/mavlink.h"
}

// ---------------- DroneStatus ----------------
struct DroneStatus {
    int sysid{0};
    std::string cdrone_id;

    std::chrono::steady_clock::time_point last_hb;
    uint8_t system_status = 0;

    // position
    int32_t lat_e7 = 0;
    int32_t lon_e7 = 0;
    int32_t alt_mm = 0;

    // ack tracking
    int last_command = -1;
    int last_result = -1;

    // hub extensions
    int battery_percent = -1; 
    int hb_status = 0;
    int weapon_readiness = 0;
    int weapon_engaged = 0;

    // UDP source
    sockaddr_in src_addr{};
    bool has_addr = false;
};

// ---------------- MavlinkBridge ----------------
class MavlinkBridge {
public:
    MavlinkBridge(const std::string &server_ip, int server_port, DroneManager &mgr);
    ~MavlinkBridge();

    DroneManager& get_manager() { return manager; }

    // high-level wrappers
    void request_stream(int sysid);
    void set_mode_guided(int sysid);
    void arm(int sysid);
    void rtl(int sysid);
    void send_waypoint(int sysid, double lat, double lon, double alt);
    void takeoff(int sysid, float altitude);
    void circle(int sysid, double lat, double lon, double alt, int turns, float radius);
    void reposition(int sysid, double lat_target, double lon_target, double alt_target, double v);

    // ack wait
    int wait_for_ack(int sysid, int command, int timeout_ms);

    // telemetry snapshot
    std::map<int, DroneStatus> snapshot_status();

private:
    // helpers
    int send_to_drone(const sockaddr_in &dst, const uint8_t *buf, size_t len);
    void track_pending_command(int sysid, int command_id);
    std::string ack_result_to_string(int result);

    void update_sys_status(int sysid, uint8_t status);
    void update_position(int sysid, int32_t lat_e7, int32_t lon_e7, int32_t alt_mm);
    void update_ack(int sysid, int command, int result);

    // threads
    void hb_loop();
    void recv_loop();

    DroneManager &manager;                        // shared discovery/id mapper
    std::map<int, DroneStatus> drone_status;      // live telemetry

    int sock_;
    sockaddr_in server_addr_{};
    std::thread recv_thread_, hb_thread_;
    std::atomic<bool> running_{false};

    std::condition_variable cv;
    std::mutex cv_mtx, status_mtx;

    int last_ack_sysid{-1};
    int last_ack_command{-1};
    int last_ack_result{-1};

    // Track only commands we sent
    std::set<std::pair<int,int>> pending_acks;  
    std::mutex ack_mtx;
};

