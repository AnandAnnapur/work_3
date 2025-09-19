// src/core/state_machine.h
#pragma once
#include <string>

enum class MissionState {
    IDLE,
    LAUNCHED,
    ARMED,
    MOVE_TO_TARGET,
    GUIDED,
    TERMINAL_MODE,
    CHASE_MODE,
    MISSION_COMPLETE,
    MISSION_ABORTED
};

class StateMachine {
public:
    StateMachine();
    MissionState get_current_state() const;
    void set_state(MissionState new_state);
    std::string state_to_string(MissionState state) const;

private:
    MissionState m_current_state;
};
