#include "robot_control/components/steps/dribbler_step.hpp"

namespace luhsoccer::robot_control {

std::pair<AbstractStep::StepState, robot_interface::RobotCommand> DribblerStep::calcCommandMessage(
    const ComponentData& /*comp_data*/) const {
    robot_interface::RobotCommand cmd;
    cmd.dribbler_mode = this->mode;
    return {AbstractStep::StepState::FINISHED, cmd};
}

DribblerStep::DribblerStep(robot_interface::DribblerMode mode) : mode(mode) {}

}  // namespace luhsoccer::robot_control