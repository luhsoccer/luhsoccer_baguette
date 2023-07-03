
#include <utility>

#include "local_planner_components/steps/dribbler_step.hpp"
#include "logger/logger.hpp"
namespace luhsoccer::local_planner {

std::pair<AbstractStep::StepState, robot_interface::RobotCommand> DribblerStep::calcCommandMessage(
    const std::shared_ptr<const transform::WorldModel>& /*wm*/, const TaskData& /*td*/,
    const RobotIdentifier& /*robot*/, const std::shared_ptr<AvoidanceManager>& /*am*/,
    const time::TimePoint /*time*/) const {
    robot_interface::RobotCommand cmd;
    cmd.dribbler_mode = this->mode;
    return {AbstractStep::StepState::FINISHED, cmd};
}

DribblerStep::DribblerStep(robot_interface::DribblerMode mode) : mode(mode) {}

}  // namespace luhsoccer::local_planner