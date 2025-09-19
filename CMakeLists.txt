cmake_minimum_required(VERSION 3.10)
project(DevZombeeSystem)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Find Dependencies ---
find_package(OpenSSL REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)

# --- Include Directories ---
include_directories(
    src
    # Add path to nlohmann/json header
    # e.g., ${CMAKE_CURRENT_SOURCE_DIR}/libs/nlohmann
    # Add path to MAVLink library
    # e.g., ${CMAKE_CURRENT_SOURCE_DIR}/libs/c_library_v2
    /home/zulu/libs/c_library_v2/
)

# --- Source Files ---
set(COMMON_SOURCES
    src/communication/udp_handler.cpp
    src/messages/json_parser.cpp
    src/encryption/aes_handler.cpp
    src/core/timing_manager.cpp
    src/communication/drone_manager.cpp
)

# --- Drone Hub Executable ---
add_executable(drone_hub_system
    src/main_hub.cpp
    src/communication/drone_hub_server.cpp
    src/communication/mavlink_bridge.cpp
    src/core/mission_controller.cpp
    src/core/state_machine.cpp
    src/communication/cannister.cpp
    ${COMMON_SOURCES}
)

target_link_libraries(drone_hub_system
    PRIVATE
    Boost::system
    Boost::thread
    OpenSSL::SSL
    OpenSSL::Crypto
    Threads::Threads
)

# --- Virtual C2 Server Executable ---
add_executable(virtual_c2_server
    src/main_server.cpp
    src/testing/virtual_server.cpp
    ${COMMON_SOURCES}
)

target_link_libraries(virtual_c2_server
    PRIVATE
    Boost::system
    Boost::thread
    OpenSSL::SSL
    OpenSSL::Crypto
    Threads::Threads
)

# --- Install Commands (Optional) ---
install(TARGETS drone_hub_system virtual_c2_server DESTINATION bin)
