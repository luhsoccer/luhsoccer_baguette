
#include "robot_control/components/steps/kick_step.hpp"
#include "config/robot_control_config.hpp"
#include "robot_control/components/component_data.hpp"

namespace luhsoccer::robot_control {
std::pair<AbstractStep::StepState, robot_interface::RobotCommand> KickStep::calcCommandMessage(
    const ComponentData& comp_data) const {
    robot_interface::KickCommand kick_cmd;
    constexpr double MAX_VEL = 6.5;
    double velocity = this->velocity.val(comp_data);
    velocity = std::min(MAX_VEL, std::max(0.0, velocity));
    if (velocity > 5.9) velocity += robotControlConfig().kick_bonus;
    kick_cmd.velocity = velocity;
    kick_cmd.type = this->chip.val(comp_data) ? robot_interface::KickType::CHIP : robot_interface::KickType::KICK;

    // TODO this was never used, since the robots only ever received the kick velocity
    /*
    double voltage = (kick_velocity - robotControlConfig().kick_velocity_offset) / robotControlConfig().kick_voltage_k;

    kick_cmd.cap_voltage = std::min(robotControlConfig().kick_max_voltage.val(),
                                    std::max(robotControlConfig().kick_min_voltage.val(), static_cast<int>(voltage)));
                                    */

    kick_cmd.execute_time = this->execute_time;
    robot_interface::RobotCommand command;
    command.kick_command = kick_cmd;

    auto ally_data = comp_data.wm->getAllyRobotData(comp_data.robot);
    if (!wait.val(comp_data)) return {AbstractStep::StepState::FINISHED, command};

    if (ally_data.has_value()) {
        switch (this->execute_time) {
            default:
            case robot_interface::KickExecuteTime::NOW:
                if (ally_data->ball_in_dribbler) {
                    return {AbstractStep::StepState::RUNNING, command};
                } else {
                    return {AbstractStep::StepState::FINISHED, command};
                }
                break;
            case robot_interface::KickExecuteTime::WHEN_BALL_IN_DRIBBLER: {
                bool had_ball = this->getCookie<HadBallFlagT>(comp_data.td, "had_ball").has_value();
                if (had_ball && !ally_data->ball_in_dribbler) {
                    return {AbstractStep::StepState::FINISHED, command};
                } else if (!had_ball && ally_data->ball_in_dribbler) {
                    this->setCookie(comp_data.td, "had_ball", HadBallFlagT());
                }
                return {AbstractStep::StepState::RUNNING, command};
            } break;
        }
    } else {
        return {AbstractStep::StepState::ERROR, command};
    }
}

std::pair<AbstractStep::StepState, robot_interface::RobotCommand> ChangeKickerModeStep::calcCommandMessage(
    const ComponentData& comp_data) const {
    robot_interface::RobotCommand command;
    robot_interface::KickCommand kick_cmd;
    kick_cmd.type = this->chip.val(comp_data) ? robot_interface::KickType::CHIP : robot_interface::KickType::KICK;
    command.kick_command = kick_cmd;
    return {AbstractStep::StepState::FINISHED, command};
}

}  // namespace luhsoccer::robot_control
