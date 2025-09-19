// src/core/timing_manager.h
#pragma once

#include <chrono>

class TimingManager {
public:
    // Frequency in Hz
    explicit TimingManager(double frequency);
    void wait_for_next_tick();

private:
    std::chrono::steady_clock::time_point m_next_tick;
    std::chrono::nanoseconds m_period;
};


