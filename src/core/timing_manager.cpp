// src/core/timing_manager.cpp
#include "timing_manager.h"
#include <thread>

TimingManager::TimingManager(double frequency) {
    if (frequency > 0) {
        m_period = std::chrono::nanoseconds(static_cast<long long>(1.0e9 / frequency));
    } else {
        m_period = std::chrono::nanoseconds(0);
    }
    m_next_tick = std::chrono::steady_clock::now() + m_period;
}

void TimingManager::wait_for_next_tick() {
    std::this_thread::sleep_until(m_next_tick);
    m_next_tick += m_period;
}
