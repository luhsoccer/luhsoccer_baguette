#include "connections/simulation.hpp"
#include "simulation_interface/simulation_interface.hpp"
#include "visit.hpp"

namespace luhsoccer::robot_interface {

namespace {
void serializeAffine2(proto::Vector3f* result, const Eigen::Affine2d& affine) {
    result->set_x(static_cast<float>(affine.translation().x()));
    result->set_y(static_cast<float>(affine.translation().y()));
    result->set_z(static_cast<float>(Eigen::Rotation2Dd(affine.rotation()).angle()));
}

void serializeVector3(proto::Vector3f* result, const Eigen::Vector3d& vector) {
    result->set_x(static_cast<float>(vector.x()));
    result->set_y(static_cast<float>(vector.y()));
    result->set_z(static_cast<float>(vector.z()));
}

proto::RobotCommand serializeRobotCommand(const RobotCommand& cmd) {
    proto::RobotCommand proto_cmd;
    proto_cmd.set_dribbler_mode(proto::RobotCommand::DribblerMode::RobotCommand_DribblerMode_OFF);
    if (cmd.dribbler_mode) {
        switch (cmd.dribbler_mode.value()) {
            case DribblerMode::LOW:
                proto_cmd.set_dribbler_mode(proto::RobotCommand::DribblerMode::RobotCommand_DribblerMode_LOW);
                break;
            case DribblerMode::HIGH:
                proto_cmd.set_dribbler_mode(proto::RobotCommand::DribblerMode::RobotCommand_DribblerMode_HIGH);
                break;
            default:
                break;
        }
    }

    if (cmd.kick_command) {
        const auto kick_command = cmd.kick_command.value();
        auto proto_kick_command = proto_cmd.mutable_kick_command();
        proto_kick_command->set_kick_velocity(static_cast<float>(kick_command.kick_velocity));
        proto_kick_command->set_chip_velocity(static_cast<float>(kick_command.chip_velocity));
        if (kick_command.cap_voltage) {
            proto_kick_command->set_cap_voltage(kick_command.cap_voltage.value());
        }
        switch (kick_command.execute_time) {
            case KickExecuteTime::NOW:
                proto_kick_command->set_execute_time(proto::KickCommand::ExecuteTime::KickCommand_ExecuteTime_NOW);
                break;
            case KickExecuteTime::WHEN_BALL_IN_DRIBBLER:
                proto_kick_command->set_execute_time(
                    proto::KickCommand::ExecuteTime::KickCommand_ExecuteTime_WHEN_BALL_IN_DRIBBLER);
                break;
        }
    }

    if (cmd.move_command) {
        auto move_command = cmd.move_command.value();
        auto proto_move_cmd = proto_cmd.mutable_move_command();

        auto visitor =
            overload{[&](const RobotPositionControl& control) {
                         auto proto_control = proto_move_cmd->mutable_robot_position_control();

                         serializeVector3(proto_control->mutable_desired_velocity(), control.desired_velocity);
                         serializeAffine2(proto_control->mutable_desired_position(), control.desired_position);
                     },
                     [&](const RobotVelocityControl& control) {
                         auto proto_control = proto_move_cmd->mutable_robot_velocity_control();
                         serializeVector3(proto_control->mutable_desired_velocity(), control.desired_velocity);
                     }};
        std::visit(visitor, move_command);
    }

    return proto_cmd;
}
}  // namespace

SimulationPacketBuilder::SimulationPacketBuilder(SimulationConnection& connection)
    : connection(connection), send_blue(false), send_yellow(false) {}

void SimulationPacketBuilder::addMessage(const RobotCommandWrapper& cmd) {
    auto command = std::make_unique<proto::RobotCommand>(serializeRobotCommand(cmd.cmd)).release();
    command->set_id(cmd.id);
    command->set_time_stamp(cmd.time_point.asNSec());

    if (cmd.color == TeamColor::BLUE) {
        send_blue = true;
        blue_control.mutable_robot_commands()->AddAllocated(command);
    } else if (cmd.color == TeamColor::YELLOW) {
        send_yellow = true;
        yellow_control.mutable_robot_commands()->AddAllocated(command);
    }
}

std::vector<RobotFeedbackWrapper> SimulationPacketBuilder::buildAndSend() {
    std::vector<RobotFeedbackWrapper> result;

    if (send_blue) {
        connection.send(blue_control, TeamColor::BLUE);
    }
    if (send_yellow) {
        connection.send(yellow_control, TeamColor::YELLOW);
    }

    std::lock_guard guard(connection.last_feedbacks_mutex);

    for (const auto& feedback : connection.last_feedbacks) {
        RobotFeedbackWrapper wrapper{};
        wrapper.color = feedback.is_blue() ? TeamColor::BLUE : TeamColor::YELLOW;
        wrapper.id = feedback.id();
        wrapper.feedback.time_stamp = time::Clock::time_point{std::chrono::nanoseconds(feedback.time_stamp())};

        if (feedback.has_robot_position()) {
            wrapper.feedback.position = {feedback.robot_position().x(), feedback.robot_position().y(),
                                         feedback.robot_position().z()};
        }

        if (feedback.has_robot_velocity()) {
            wrapper.feedback.velocity = {feedback.robot_velocity().x(), feedback.robot_velocity().y(),
                                         feedback.robot_velocity().z()};
        }

        if (feedback.has_robot_has_ball()) {
            wrapper.feedback.has_ball = feedback.robot_has_ball();
        }

        if (feedback.has_telemetry()) {
            RobotTelemetry telemetry{};
            telemetry.battery_voltage = feedback.telemetry().battery_voltage();
            telemetry.capacitor_voltage = feedback.telemetry().cap_voltage();
            wrapper.feedback.telemetry = telemetry;
        }
        result.push_back(wrapper);
    }
    connection.last_feedbacks.clear();

    return result;
}

SimulationConnection::SimulationConnection(simulation_interface::SimulationInterface& interface)
    : interface(interface) {
    this->interface.setRobotOutput([this](const proto::RobotFeedback& feedback, TeamColor /*color*/) {
        std::lock_guard guard(this->last_feedbacks_mutex);
        this->last_feedbacks.push_back(feedback);
    });
}

void SimulationConnection::send(const proto::RobotControl& cmd, TeamColor color) {
    this->interface.sendRobotCommand(cmd, color);
}

}  // namespace luhsoccer::robot_interface
