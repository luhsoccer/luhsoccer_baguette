
#include "local_planner_components/steps/kick_step.hpp"
#include "local_planner/local_planner_util.hpp"
#include "local_planner/skills/abstract_step.hpp"
#include "local_planner/skills/skill_util.hpp"
#include "robot_interface/robot_interface_types.hpp"

namespace luhsoccer::local_planner {
std::pair<AbstractStep::StepState, robot_interface::RobotCommand> KickStep::calcCommandMessage(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
    const std::shared_ptr<AvoidanceManager>& /*am*/, time::TimePoint /*time*/) const {
    robot_interface::KickCommand kick_cmd;
    constexpr double MAX_VEL = 6.5;
    kick_cmd.kick_velocity = std::min(MAX_VEL, std::max(0.0, velocity.val(wm, td)));

    double voltage = (velocity.val(wm, td) - localPlannerConfig().step_kick_velocity_offset) /
                     localPlannerConfig().step_kick_voltage_k;
    kick_cmd.cap_voltage = std::min(localPlannerConfig().step_kick_voltage_max.val(),
                                    std::max(localPlannerConfig().step_kick_voltage_min.val(), voltage));

    kick_cmd.execute_time = this->execute_time;
    robot_interface::RobotCommand command;
    command.kick_command = kick_cmd;

    auto ally_data = wm->getAllyRobotData(robot);
    if (!wait.val(wm, td)) return {AbstractStep::StepState::FINISHED, command};

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
                bool had_ball = this->getCookie<HadBallFlagT>(td, "had_ball").has_value();
                if (had_ball && !ally_data->ball_in_dribbler) {
                    return {AbstractStep::StepState::FINISHED, command};
                } else if (!had_ball && ally_data->ball_in_dribbler) {
                    this->setCookie(td, "had_ball", HadBallFlagT());
                }
                return {AbstractStep::StepState::RUNNING, command};
            } break;
        }
    } else {
        return {AbstractStep::StepState::ERROR, command};
    }
}

}  // namespace luhsoccer::local_planner
