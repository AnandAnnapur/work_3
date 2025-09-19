#pragma once

#include <iostream>
#include <thread>
#include <mutex>
#include <map>
#include <set>
#include <chrono>
#include <string>
#include <condition_variable>
#include <netinet/in.h>
#include <sstream>
#include <iomanip>

class DroneManager {
public:
    // --- Discovery / ID assignment utilities ---
    void discover(int sysid);
    void assign_ids();
    std::string get_cdrone_id(int sysid);
    std::set<int> get_discovered();

private:
    std::string format_number(int num);

    std::set<int> discovered;
    std::set<int> seen;
    std::map<int, std::string> id_map;
    std::mutex mtx;
};
