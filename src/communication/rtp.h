#pragma once
#include <string>

class RTSPManager {
public:
    RTSPManager(int rtspPort = 8554);

    // Auto scan /dev/video* and /dev/ttyVideo*
    void autoAssignDevices();

    // Add raw pipeline
    void addDroneStream(const std::string &endpoint,
                        const std::string &pipeline);

    // Map a specific drone ID to an RTSP endpoint
    void addDroneForId(const std::string &cdrone_id,
                       const std::string &device,
                       bool testsrcFallback = true);

    // Auto detect input type (rtsp, udp, v4l2, ttyVideo)
    void addAutoInput(const std::string &cdrone_id,
                      const std::string &input,
                      int extra = 0);

    // Load config from YAML file
    void loadConfigYAML(const std::string &filename);

    ~RTSPManager();

private:
    struct Impl;
    Impl *impl;
};

