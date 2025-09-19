// src/core/state_machine.cpp
#include "state_machine.h"

StateMachine::StateMachine() : m_current_state(MissionState::IDLE) {}

MissionState StateMachine::get_current_state() const {
    return m_current_state;
}

void StateMachine::set_state(MissionState new_state) {
    if (m_current_state != new_state) {
        m_current_state = new_state;
    }
}

std::string StateMachine::state_to_string(MissionState state) const {
    switch(state) {
        case MissionState::IDLE: return "IDLE";
        case MissionState::LAUNCHED: return "LAUNCHED";
        case MissionState::ARMED: return "ARMED";
        case MissionState::MOVE_TO_TARGET: return "MOVE_TO_TARGET";
        case MissionState::GUIDED: return "GUIDED";
        case MissionState::TERMINAL_MODE: return "TERMINAL_MODE";
        case MissionState::CHASE_MODE: return "CHASE_MODE";
        case MissionState::MISSION_COMPLETE: return "MISSION_COMPLETE";
        case MissionState::MISSION_ABORTED: return "MISSION_ABORTED";
        default: return "UNKNOWN";
    }
}
