#pragma once

#include <time/time.hpp>
#include <optional>
#include <variant>
#include <Eigen/Geometry>
#include "robot_identifier.hpp"

namespace luhsoccer::robot_interface {

enum class KickExecuteTime {
    NOW,
    WHEN_BALL_IN_DRIBBLER,
};

struct KickCommand {
    double kick_velocity{0.0};
    double chip_velocity{0.0};
    std::optional<int> cap_voltage{};
    KickExecuteTime execute_time{KickExecuteTime::NOW};
};

struct RobotPositionControl {
    Eigen::Affine2d desired_position{};
    Eigen::Vector3d desired_velocity{};
    time::TimePoint time_execute{};
};

struct RobotVelocityControl {
    Eigen::Vector3d desired_velocity{};
};

enum MoveRestriction { HALT, STOP, FREE };

struct StopMovement {};
struct ReceiveBall {
    // dummy for onboard controlling
    time::TimePoint stamp;
};

using SpecialMode = std::variant<StopMovement, MoveRestriction, ReceiveBall>;

using MoveCommand = std::variant<RobotPositionControl, RobotVelocityControl>;

enum class DribblerMode { OFF, LOW, HIGH };

struct RobotCommand {
    time::TimePoint time_sent{};
    std::optional<SpecialMode> special_mode{};
    std::optional<MoveCommand> move_command{};
    std::optional<KickCommand> kick_command{};
    std::optional<DribblerMode> dribbler_mode{};
};

enum class ConnectionQuality { NO_CONNECTION, BAD, GOOD };

struct RobotTelemetry {
    double battery_voltage{0.0};
    double capacitor_voltage{0.0};
    ConnectionQuality quality{ConnectionQuality::NO_CONNECTION};
    bool light_barrier_working{false};
};

struct RobotTelemetryRF {
    double rssi_robot{0.0};
    double rssi_base_station{0.0};
    double frequency{0.0};
};

struct RobotFeedback {
    time::TimePoint time_stamp;
    std::optional<RobotTelemetry> telemetry;
    std::optional<Eigen::Vector3d> position;
    std::optional<Eigen::Vector3d> velocity;
    std::optional<bool> has_ball;
    std::optional<uint8_t> last_special_command_id;
    std::optional<RobotTelemetryRF> telemetry_rf;
};

std::ostream& operator<<(std::ostream& os, const RobotFeedback& feedback);

std::ostream& operator<<(std::ostream& os, const RobotCommand& feedback);

}  // namespace luhsoccer::robot_interface