#include "robot_control/components/steps/condition_step.hpp"

#include "robot_control/components/component_data.hpp"

namespace luhsoccer::robot_control {

std::pair<AbstractStep::StepState, robot_interface::RobotCommand> ConditionStep::calcCommandMessage(
    const ComponentData& comp_data) const {
    std::optional<unsigned int> current_instruction =
        this->getCookie<unsigned int>(comp_data.td, "current_instruction");
    std::optional<bool> condition_val_on_check = this->getCookie<bool>(comp_data.td, "condition_val_on_check");

    if (!condition_val_on_check.has_value()) {
        condition_val_on_check = condition.val(comp_data);
        this->setCookie(comp_data.td, "condition_val_on_check", condition_val_on_check.value());
    }

    if (!current_instruction.has_value()) {
        current_instruction = 0;
    }

    std::pair<AbstractStep::StepState, robot_interface::RobotCommand> res;
    AbstractStep::StepState state = AbstractStep::StepState::RUNNING;
    robot_interface::RobotCommand cmd = robot_interface::RobotCommand();
    if (condition_val_on_check.value() && current_instruction.value() < if_steps.size()) {
        res = this->if_steps.at(current_instruction.value())->calcCommandMessage(comp_data);
        cmd = res.second;
    } else if (!condition_val_on_check.value() && current_instruction.value() < else_steps.size()) {
        res = this->else_steps.at(current_instruction.value())->calcCommandMessage(comp_data);
        cmd = res.second;
    } else {
        state = AbstractStep::StepState::FINISHED;
        res.first = AbstractStep::StepState::FINISHED;
    }

    if (res.first != AbstractStep::StepState::FINISHED) state = res.first;

    if (res.first == AbstractStep::StepState::FINISHED) {
        this->setCookie(comp_data.td, "current_instruction", (unsigned int)current_instruction.value() + 1);
        if ((condition_val_on_check.value() && current_instruction.value() + 1 >= if_steps.size()) ||
            (!condition_val_on_check.value() && current_instruction.value() + 1 >= else_steps.size())) {
            state = AbstractStep::StepState::FINISHED;
        }
    } else {
        this->setCookie(comp_data.td, "current_instruction", current_instruction.value());
    }

    return {state, cmd};
}

ConditionStep::ConditionStep(BoolComponentParam condition) : condition(std::move(condition)) {}

}  // namespace luhsoccer::robot_control