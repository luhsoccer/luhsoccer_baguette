
#include <utility>

#include "local_planner_components/steps/condition_step.hpp"
#include "logger/logger.hpp"
namespace luhsoccer::local_planner {

std::pair<AbstractStep::StepState, robot_interface::RobotCommand> ConditionStep::calcCommandMessage(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
    const std::shared_ptr<AvoidanceManager>& am, const time::TimePoint time) const {
    std::optional<unsigned int> current_instruction = this->getCookie<unsigned int>(td, "current_instruction");
    std::optional<bool> condition_val_on_check = this->getCookie<bool>(td, "condition_val_on_check");

    if (!condition_val_on_check.has_value()) {
        condition_val_on_check = condition.val(wm, td);
        this->setCookie(td, "condition_val_on_check", condition_val_on_check.value());
    }

    if (!current_instruction.has_value()) {
        current_instruction = 0;
    }

    std::pair<AbstractStep::StepState, robot_interface::RobotCommand> res;
    AbstractStep::StepState state = AbstractStep::StepState::RUNNING;
    robot_interface::RobotCommand cmd = robot_interface::RobotCommand();
    if (condition_val_on_check.value() && current_instruction.value() < if_steps.size()) {
        res = this->if_steps.at(current_instruction.value())->calcCommandMessage(wm, td, robot, am, time);
        cmd = res.second;
    } else if (!condition_val_on_check.value() && current_instruction.value() < else_steps.size()) {
        res = this->else_steps.at(current_instruction.value())->calcCommandMessage(wm, td, robot, am, time);
        cmd = res.second;
    } else {
        state = AbstractStep::StepState::FINISHED;
        res.first = AbstractStep::StepState::FINISHED;
    }

    if (res.first != AbstractStep::StepState::FINISHED) state = res.first;

    if (res.first == AbstractStep::StepState::FINISHED) {
        this->setCookie(td, "current_instruction", (unsigned int)current_instruction.value() + 1);
        if ((condition_val_on_check.value() && current_instruction.value() + 1 >= if_steps.size()) ||
            (!condition_val_on_check.value() && current_instruction.value() + 1 >= else_steps.size())) {
            state = AbstractStep::StepState::FINISHED;
        }
    } else {
        this->setCookie(td, "current_instruction", current_instruction.value());
    }

    return {state, cmd};
}

[[nodiscard]] std::vector<Marker> ConditionStep::getVisualizationMarkers(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
    const time::TimePoint time) const {
    std::optional<unsigned int> current_instruction = this->getCookie<unsigned int>(td, "current_instruction");
    std::optional<bool> condition_val_on_check = this->getCookie<bool>(td, "condition_val_on_check");
    if (!condition_val_on_check.has_value()) return {};

    if (!current_instruction.has_value()) {
        current_instruction = 0;
    }

    if (condition_val_on_check.value() && current_instruction.value() < if_steps.size()) {
        return this->if_steps.at(current_instruction.value())->getVisualizationMarkers(wm, td, robot, time);
    } else if (!condition_val_on_check.value() && current_instruction.value() < else_steps.size()) {
        return this->else_steps.at(current_instruction.value())->getVisualizationMarkers(wm, td, robot, time);
    } else {
        return {};
    }
}

ConditionStep::ConditionStep(BoolComponentParam condition) : condition(std::move(condition)) {}

}  // namespace luhsoccer::local_planner