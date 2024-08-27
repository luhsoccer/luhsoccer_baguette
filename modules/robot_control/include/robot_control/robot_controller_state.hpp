#pragma once

namespace luhsoccer::robot_control {
enum class RobotControllerState { IDLE, RUNNING, OUT_OF_FIELD, ERROR, CREATED, OFFLINE, FINISHED, ABORTED };

inline std::string format_as(const RobotControllerState& mode) {
    switch (mode) {
        case RobotControllerState::IDLE:
            return "IDLE";
        case RobotControllerState::RUNNING:
            return "RUNNING";
        case RobotControllerState::OUT_OF_FIELD:
            return "OUT_OF_FIELD";
        case RobotControllerState::ERROR:
            return "ERROR";
        case RobotControllerState::CREATED:
            return "CREATED";
        case RobotControllerState::OFFLINE:
            return "OFFLINE";
        case RobotControllerState::FINISHED:
            return "FINISHED";
        case RobotControllerState::ABORTED:
            return "ABORTED";
    }
}
};  // namespace luhsoccer::robot_control