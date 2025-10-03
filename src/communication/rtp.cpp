#include "rtp.h"
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <thread>
#include <vector>
#include <filesystem>
#include <regex>
#include <iostream>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

struct RTSPManager::Impl {
    GstRTSPServer *server = nullptr;
    GstRTSPMountPoints *mounts = nullptr;
    GMainLoop *loop = nullptr;
    std::thread loopThread;
};

// ---- Utility: check if /dev/videoN is a valid capture device ----
static bool isVideoCaptureDevice(const std::string &devicePath, std::string &nameOut) {
    int fd = open(devicePath.c_str(), O_RDWR);
    if (fd < 0) return false;

    v4l2_capability cap {};
    bool result = false;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == 0) {
        nameOut = reinterpret_cast<char*>(cap.card);
        if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) &&
            (cap.capabilities & V4L2_CAP_STREAMING)) {
            result = true;
        }
    }
    close(fd);
    return result;
}

RTSPManager::RTSPManager(int rtspPort) {
    gst_init(nullptr, nullptr);
    impl = new Impl();

    impl->server = gst_rtsp_server_new();
    gst_rtsp_server_set_address(impl->server, "0.0.0.0");  // ✅ listen on all network interfaces
    gst_rtsp_server_set_service(impl->server, std::to_string(rtspPort).c_str());
    impl->mounts = gst_rtsp_server_get_mount_points(impl->server);

    gst_rtsp_server_attach(impl->server, nullptr);
    g_print("✅ RTSP server running at rtsp://0.0.0.0:%d\n", rtspPort);

    impl->loop = g_main_loop_new(nullptr, FALSE);
    impl->loopThread = std::thread([this]() { g_main_loop_run(impl->loop); });
}

// ---- Core helper ----
void RTSPManager::addDroneStream(const std::string &endpoint,
                                 const std::string &pipeline) {
    GstRTSPMediaFactory *factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory, pipeline.c_str());
    gst_rtsp_media_factory_set_shared(factory, TRUE);

    gst_rtsp_mount_points_add_factory(impl->mounts, endpoint.c_str(), factory);

    g_print("🔹 Added stream → rtsp://127.0.0.1:8554%s\n", endpoint.c_str());
}

// ---- Auto input: RTSP, UDP, V4L2, ttyVideo ----
void RTSPManager::addAutoInput(const std::string &cdrone_id,
                               const std::string &input,
                               int extra) {
    std::string endpoint = "/dronevideo" + cdrone_id;
    std::string pipeline;

    if (input.rfind("rtsp://", 0) == 0) {
        pipeline = "( rtspsrc location=" + input +
                   " latency=100 ! rtph264depay ! h264parse "
                   "! rtph264pay name=pay0 pt=96 )";
        g_print("🎥 RTSP input: %s\n", input.c_str());
    }
    else if (input.rfind("udp://", 0) == 0) {
        auto pos = input.find("://");
        std::string addrPort = input.substr(pos + 3);
        auto colon = addrPort.find(":");
        std::string addr = addrPort.substr(0, colon);
        int port = std::stoi(addrPort.substr(colon + 1));

        pipeline = "( udpsrc address=" + addr + " port=" + std::to_string(port) +
                   " caps=\"application/x-rtp,media=video,encoding-name=H264,payload=96\" "
                   "! rtph264depay ! h264parse ! rtph264pay name=pay0 pt=96 )";
        g_print("🎥 Multicast input: %s:%d\n", addr.c_str(), port);
    }
    else if (input.find("/dev/ttyVideo") == 0) {
        pipeline = "( filesrc location=" + input +
                   " ! h264parse ! rtph264pay name=pay0 pt=96 )";
        g_print("🎥 Serial-video input: %s\n", input.c_str());
    }
    else if (input.find("/dev/video") == 0) {
        pipeline = "( v4l2src device=" + input +
                   " ! videoconvert "
                   "! x264enc tune=zerolatency bitrate=500 speed-preset=superfast "
                   "! rtph264pay name=pay0 pt=96 )";
        g_print("🎥 V4L2 input: %s\n", input.c_str());
    }
    else {
        g_print("⚠️ Unknown input type: %s\n", input.c_str());
        return;
    }

    if (!pipeline.empty()) {
        addDroneStream(endpoint, pipeline);
    }
}

// ---- Auto assign devices with duplicate filtering ----
void RTSPManager::autoAssignDevices() {
    std::vector<std::string> devPaths;

    for (const auto &entry : std::filesystem::directory_iterator("/dev")) {
        std::string path = entry.path();
        if (std::regex_match(path, std::regex("/dev/video[0-9]+")) ||
            std::regex_match(path, std::regex("/dev/ttyVideo[0-9]+"))) {
            devPaths.push_back(path);
        }
    }

    std::sort(devPaths.begin(), devPaths.end(), [](const std::string &a, const std::string &b) {
        auto extractNum = [](const std::string &s) {
            size_t pos = s.find_last_not_of("0123456789");
            return std::stoi(s.substr(pos + 1));
        };
        return extractNum(a) < extractNum(b);
    });

    int index = 1;
    bool found = false;

    for (const auto &path : devPaths) {
        std::string name;
        if (path.find("/dev/video") == 0) {
            if (!isVideoCaptureDevice(path, name)) {
                g_print("⚠️ Skipping non-capture device: %s\n", path.c_str());
                continue;
            }
            g_print("🎥 AutoAssign found V4L2: %s (%s)\n", path.c_str(), name.c_str());
        } else if (path.find("/dev/ttyVideo") == 0) {
            g_print("🎥 AutoAssign found SerialVideo: %s\n", path.c_str());
        }

        addAutoInput(std::to_string(index++), path);
        found = true;
    }

    if (!found) {
        g_print("⚠️ No valid devices found, adding testsrc fallback...\n");
        std::string pipeline =
            "( videotestsrc is-live=true "
            "! x264enc tune=zerolatency bitrate=500 speed-preset=superfast "
            "! rtph264pay name=pay0 pt=96 )";
        addDroneStream("/dronevideo1", pipeline);
    }
}

// ---- Old: map drone ID to endpoint ----
void RTSPManager::addDroneForId(const std::string &cdrone_id,
                                const std::string &device,
                                bool testsrcFallback) {
    std::string endpoint = "/dronevideo" + cdrone_id;
    std::string pipeline;

    if (!device.empty()) {
        pipeline =
            "( v4l2src device=" + device +
            " ! videoconvert "
            "! x264enc tune=zerolatency bitrate=500 speed-preset=superfast "
            "! rtph264pay name=pay0 pt=96 )";
    } else if (testsrcFallback) {
        pipeline =
            "( videotestsrc is-live=true "
            "! x264enc tune=zerolatency bitrate=500 speed-preset=superfast "
            "! rtph264pay name=pay0 pt=96 )";
    }

    if (!pipeline.empty()) {
        addDroneStream(endpoint, pipeline);
    }
}

// ---- Load config from YAML ----
void RTSPManager::loadConfigYAML(const std::string &filename) {
    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(filename);
    } catch (const std::exception &e) {
        g_print("⚠️ Could not open YAML config file: %s (%s)\n",
                filename.c_str(), e.what());
        return;
    }

    int port = cfg["rtsp_port"] ? cfg["rtsp_port"].as<int>() : 8554;
    g_print("✅ Loaded config. RTSP Port = %d\n", port);

    int index = 1;
    if (cfg["streams"]) {
        for (auto stream : cfg["streams"]) {
            std::string id = stream["id"] ? stream["id"].as<std::string>() : std::to_string(index++);
            std::string input = stream["input"] ? stream["input"].as<std::string>() : "";
            std::string res = stream["resolution"] ? stream["resolution"].as<std::string>() : "640x480";
            int fps = stream["framerate"] ? stream["framerate"].as<int>() : 30;
            int bitrate = stream["bitrate"] ? stream["bitrate"].as<int>() : 1000;
            std::string encoder = stream["encoder"] ? stream["encoder"].as<std::string>() : "x264";
            std::string preset = stream["preset"] ? stream["preset"].as<std::string>() : "superfast";
            std::string tune = stream["tune"] ? stream["tune"].as<std::string>() : "zerolatency";
            int latency = stream["latency"] ? stream["latency"].as<int>() : 100;

            std::string endpoint = "/dronevideo" + id;
            std::string pipeline;

            std::string width = res.substr(0, res.find('x'));
            std::string height = res.substr(res.find('x') + 1);

            if (input.find("/dev/video") == 0) {
                pipeline =
                    "( v4l2src device=" + input +
                    " ! video/x-raw,width=" + width +
                    ",height=" + height +
                    ",framerate=" + std::to_string(fps) + "/1 "
                    "! videoconvert "
                    "! " + encoder + "enc bitrate=" + std::to_string(bitrate) +
                    " speed-preset=" + preset +
                    " tune=" + tune + " "
                    "! rtph264pay name=pay0 pt=96 )";
            }
            else if (input.find("/dev/ttyVideo") == 0) {
                pipeline =
                    "( filesrc location=" + input +
                    " ! h264parse ! rtph264pay name=pay0 pt=96 )";
            }
            else if (input.rfind("rtsp://", 0) == 0) {
                pipeline =
                    "( rtspsrc location=" + input + " latency=" + std::to_string(latency) +
                    " ! rtph264depay ! h264parse ! rtph264pay name=pay0 pt=96 )";
            }
            else if (input.rfind("udp://", 0) == 0) {
                auto pos = input.find("://");
                std::string addrPort = input.substr(pos + 3);
                auto colon = addrPort.find(":");
                std::string addr = addrPort.substr(0, colon);
                int port = std::stoi(addrPort.substr(colon + 1));

                pipeline =
                    "( udpsrc address=" + addr + " port=" + std::to_string(port) +
                    " caps=\"application/x-rtp,media=video,encoding-name=H264,payload=96\" "
                    "! rtph264depay ! h264parse ! rtph264pay name=pay0 pt=96 )";
            }
            else {
                g_print("⚠️ Unknown input type: %s\n", input.c_str());
                continue;
            }

            addDroneStream(endpoint, pipeline);
        }
    }
}

RTSPManager::~RTSPManager() {
    if (impl->loop) g_main_loop_quit(impl->loop);
    if (impl->loopThread.joinable()) impl->loopThread.join();
    if (impl->mounts) g_object_unref(impl->mounts);
    if (impl->server) g_object_unref(impl->server);
    delete impl;
}

