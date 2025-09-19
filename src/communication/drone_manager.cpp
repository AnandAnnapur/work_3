#include "drone_manager.h"

// --- Discovery ---
void DroneManager::discover(int sysid) {
    std::lock_guard<std::mutex> lock(mtx);
    if (sysid <= 0 || sysid == 255) return; // ignore invalid & GCS
    discovered.insert(sysid);
    if (seen.insert(sysid).second) {
        std::cout << "[DISCOVERED] Drone sysid=" << sysid << std::endl;
    }
}

// --- Assign unique compact IDs ---
void DroneManager::assign_ids() {
    std::lock_guard<std::mutex> lock(mtx);
    int counter = 1;
    for (int sysid : discovered) {
        if (id_map.find(sysid) == id_map.end()) {
            std::string cdrone_id = "AA" + format_number(counter++);
            id_map[sysid] = cdrone_id;
            std::cout << "[REGISTER] sysid=" << sysid
                      << " -> cdrone_id=" << cdrone_id << std::endl;
        }
    }
}

// --- Lookup compact ID for sysid ---
std::string DroneManager::get_cdrone_id(int sysid) {
    std::lock_guard<std::mutex> lock(mtx);
    auto it = id_map.find(sysid);
    return (it != id_map.end()) ? it->second : "UNKNOWN";
}

// --- Return all discovered sysids ---
std::set<int> DroneManager::get_discovered() {
    std::lock_guard<std::mutex> lock(mtx);
    return discovered;
}

// --- Helper to format AA001, AA002...
std::string DroneManager::format_number(int num) {
    std::ostringstream ss;
    ss << std::setw(3) << std::setfill('0') << num;
    return ss.str();
}
