
#include "robot_control/skills/skill.hpp"

#include "robot_control/components/component_data.hpp"
#include "robot_control/skills/task_data.hpp"
#include "robot_control/components/abstract_step.hpp"

namespace luhsoccer::robot_control {

std::optional<size_t> initParamSize(const std::vector<std::string>& param_names) {
    if (param_names.size() == 1 && param_names[0] == "VARIADIC") {
        return std::nullopt;
    } else {
        return param_names.size();
    }
};

Skill::Skill(std::string name, const std::vector<std::string>& related_robot,
             const std::vector<std::string>& required_point, const std::vector<std::string>& required_double,
             const std::vector<std::string>& required_int, const std::vector<std::string>& required_bool,
             const std::vector<std::string>& required_string)
    : name(std::move(name)),
      related_robot_num(initParamSize(related_robot)),
      required_point_num(initParamSize(required_point)),
      required_double_num(initParamSize(required_double)),
      required_int_num(initParamSize(required_int)),
      required_bool_num(initParamSize(required_bool)),
      required_string_num(initParamSize(required_string)),
      related_robot(related_robot),
      required_point(required_point),
      required_double(required_double),
      required_int(required_int),
      required_bool(required_bool),
      required_string(required_string){};

std::pair<Skill::SkillState, robot_interface::RobotCommand> Skill::calcCommandMessage(const ComponentData& comp_data,
                                                                                      const size_t step_num) const {
    if (step_num >= this->steps.size()) {
        logger::Logger("skills").error("Requested step number ({:d}) is to high. Skill '{}' has only {:d} steps.",
                                       step_num, this->name, this->steps.size());
        return {Skill::SkillState::ERROR, robot_interface::RobotCommand()};
    }
    if (!this->steps[step_num])
        throw std::runtime_error(fmt::format("Step {:d} in Skill '{}' is a nullptr!", step_num, this->name));
    auto [state, command_msg] = this->steps[step_num]->calcCommandMessage(comp_data);

    switch (state) {
        case AbstractStep::StepState::RUNNING:
            return {SkillState::RUNNING, command_msg};
        case AbstractStep::StepState::FINISHED:
            if (step_num == this->steps.size() - 1) {
                return {SkillState::FINISHED, command_msg};
            } else {
                return {SkillState::NEXT_STEP, command_msg};
            }
        case AbstractStep::StepState::ABORT:
            return {SkillState::ABORT, robot_interface::RobotCommand()};
        default:
        case AbstractStep::StepState::ERROR:
            return {SkillState::ERROR, robot_interface::RobotCommand()};
    }
}

bool Skill::taskDataValid(const TaskData& td) const {
    static constexpr auto COMPARE_SIZE = [](const std::optional<size_t>& desired_size, size_t actual_size) {
        return !desired_size.has_value() || desired_size.value() == actual_size;
    };
    return COMPARE_SIZE(this->related_robot_num, td.related_robots.size()) &&
           COMPARE_SIZE(this->required_point_num, td.required_positions.size()) &&
           COMPARE_SIZE(this->required_double_num, td.required_doubles.size()) &&
           COMPARE_SIZE(this->required_int_num, td.required_ints.size()) &&
           COMPARE_SIZE(this->required_bool_num, td.required_bools.size()) &&
           COMPARE_SIZE(this->required_string_num, td.required_strings.size());
}
}  // namespace luhsoccer::robot_control