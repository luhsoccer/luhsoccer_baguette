#include "robot_interface/robot_interface_types.hpp"
#include "visit.hpp"
namespace luhsoccer::robot_interface {

std::ostream& operator<<(std::ostream& os, const RobotFeedback& feedback) {
    os << fmt::format("Feedback from time {}:", feedback.time_stamp) << std::endl;
    if (feedback.has_ball.has_value()) os << "has_ball: " << feedback.has_ball.value() << std::endl;
    if (feedback.position.has_value())
        os << fmt::format("position: ({:0.3f},{:0.3f},{:0.3f})", feedback.position->x(), feedback.position->y(),
                          feedback.position->z())
           << std::endl;
    if (feedback.velocity.has_value())
        os << fmt::format("velocity: ({:0.3f},{:0.3f},{:0.3f})", feedback.velocity->x(), feedback.velocity->y(),
                          feedback.velocity->z())
           << std::endl;
    return os;
}

std::ostream& operator<<(std::ostream& os, const RobotCommand& command) {
    os << fmt::format("Time sent: {}", command.time_sent) << std::endl;
    os << "SpecialMode: ";
    if (command.special_mode.has_value()) {
        auto visitor = overload{
            [&os](const StopMovement&) { os << "StopMovement" << std::endl; },
            [&os](const MoveRestriction& restriction) {
                switch (restriction) {
                    case MoveRestriction::HALT:
                        os << "HALT" << std::endl;
                        break;
                    case MoveRestriction::STOP:
                        os << "STOP" << std::endl;
                        break;
                    case MoveRestriction::FREE:
                        os << "FREE" << std::endl;
                        break;
                }
            },
            [&os](const ReceiveBall& receive_ball) {
                os << "ReceiveBall: time_point: " << receive_ball.stamp << std::endl;
            },
        };
        std::visit(visitor, command.special_mode.value());
    } else {
        os << std::endl;
    }
    os << "MoveCommand: ";
    if (command.move_command.has_value()) {
        auto visitor = overload{
            [&os](const RobotPositionControl& pos_control) {
                os << "Postion control: \n Position: \n"
                   << pos_control.desired_position.translation().x() << "\n"
                   << pos_control.desired_position.translation().y() << "\n"
                   << Eigen::Rotation2Dd(pos_control.desired_position.rotation()).angle() << "\n";
                ;
                os << " Velocity: \n" << pos_control.desired_velocity;
            },
            [&os](const RobotVelocityControl& vel_constol) {
                os << "Velocity control: \n" << vel_constol.desired_velocity;
            },
        };
        std::visit(visitor, command.move_command.value());
    } else {
        os << std::endl;
    }
    os << "KickCommand: ";
    if (command.kick_command.has_value()) {
        os << "Kick velocity: " << command.kick_command->kick_velocity << std::endl;
        os << "Chip velocity: " << command.kick_command->chip_velocity << std::endl;
        os << "Cap voltage: " << command.kick_command->cap_voltage.value_or(0) << std::endl;
        os << "Execute time: ";
        if (command.kick_command->execute_time == KickExecuteTime::NOW)
            os << "NOW" << std::endl;
        else
            os << "LATER" << std::endl;
    } else {
        os << std::endl;
    }
    os << "DribblerMode: ";
    if (command.dribbler_mode.has_value()) {
        switch (command.dribbler_mode.value()) {
            case DribblerMode::HIGH:
                os << "HIGH" << std::endl;
                break;
            case DribblerMode::LOW:
                os << "LOW" << std::endl;
                break;
            case DribblerMode::OFF:
                os << "OFF" << std::endl;
                break;
        }
    } else {
        os << std::endl;
    }
    return os;
}

}  // namespace luhsoccer::robot_interface