#include "communication/mavlink_bridge.h"
#include "communication/drone_hub_server.h"
#include "communication/drone_manager.h"
#include <iostream>

int main(int argc, char** argv) {
    std::string server_ip = "127.0.0.1";   // or your multiplexer/router IP
    int server_port = 14600;               // the UDP port you bind to

    DroneManager manager;
    MavlinkBridge mavlink_bridge(server_ip, server_port, manager);
        std::cout << "[MAIN] Waiting for drone heartbeats..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // 🔹 Assign compact IDs (AA001, AA002, ...)
    manager.assign_ids();
    std::this_thread::sleep_for(std::chrono::seconds(5));

    DroneHubServer hub(mavlink_bridge);
    hub.run();

    return 0;
}

