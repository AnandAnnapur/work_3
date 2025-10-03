cmake_minimum_required(VERSION 3.10)
project(DevZombeeSystem)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Find Dependencies ---
find_package(OpenSSL REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-rtsp-server-1.0)
find_package(yaml-cpp REQUIRED)

# --- Include Directories ---
include_directories(
    src
    ${GST_INCLUDE_DIRS}
    /home/anand/libs/c_library_v2/   # MAVLink
    # Add path to nlohmann/json if not system-installed
)

link_directories(${GST_LIBRARY_DIRS})

# --- Common Sources ---
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
    src/communication/rtp.cpp   # ✅ Add RTSP
    ${COMMON_SOURCES}
)

target_link_libraries(drone_hub_system
    PRIVATE
    Boost::system
    Boost::thread
    OpenSSL::SSL
    OpenSSL::Crypto
    Threads::Threads
    ${GST_LIBRARIES}   # ✅ Link GStreamer
    yaml-cpp
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

