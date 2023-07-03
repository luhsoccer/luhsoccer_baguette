
#include "local_planner/skills/skill.hpp"

#include "local_planner/skills/task.hpp"
#include "local_planner/skills/abstract_step.hpp"

namespace luhsoccer::local_planner {

Skill::Skill(std::string name, const std::vector<std::string>& related_robot,
             const std::vector<std::string>& required_point, const std::vector<std::string>& required_double,
             const std::vector<std::string>& required_int, const std::vector<std::string>& required_bool,
             const std::vector<std::string>& required_string)
    : name(std::move(name)),
      related_robot_num(related_robot.size()),
      required_point_num(required_point.size()),
      required_double_num(required_double.size()),
      required_int_num(required_int.size()),
      required_bool_num(required_bool.size()),
      required_string_num(required_string.size()),
      related_robot(related_robot),
      required_point(required_point),
      required_double(required_double),
      required_int(required_int),
      required_bool(required_bool),
      required_string(required_string){};

std::pair<Skill::SkillState, robot_interface::RobotCommand> Skill::calcCommandMessage(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
    const time::TimePoint& time, const size_t step_num, const std::shared_ptr<AvoidanceManager>& am) const {
    if (step_num >= this->steps.size()) {
        logger::Logger log("Skills");
        LOG_ERROR(log, "Requested step number ({:d}) is to high. Skill '{}' has only {:d} steps.", step_num, this->name,
                  this->steps.size());
        return {Skill::SkillState::ERROR, robot_interface::RobotCommand()};
    }
    if (!this->steps[step_num])
        throw std::runtime_error(fmt::format("Step {:d} in Skill '{}' is a nullptr!", step_num, this->name));
    auto [state, command_msg] = this->steps[step_num]->calcCommandMessage(wm, td, robot, am, time);

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

std::vector<Marker> Skill::getVisualizationMarkers(const std::shared_ptr<const transform::WorldModel>& wm,
                                                   const TaskData& td, const RobotIdentifier& robot,
                                                   const time::TimePoint& time, const size_t step_num) const {
    if (step_num >= this->steps.size()) {
        logger::Logger log("Skills");
        LOG_ERROR(log, "Requested step number ({:d}) is to high. Skill '{}' has only {:d} steps.", step_num, this->name,
                  this->steps.size());
        return {};
    }
    if (!this->steps[step_num])
        throw std::runtime_error(fmt::format("Step {:d} in Skill '{}' is a nullptr!", step_num, this->name));

    return this->steps[step_num]->getVisualizationMarkers(wm, td, robot, time);
}

bool Skill::taskDataValid(const TaskData& td) const {
    return this->related_robot_num == td.related_robots.size() &&
           this->required_point_num == td.required_positions.size() &&
           this->required_double_num == td.required_doubles.size() &&
           this->required_int_num == td.required_ints.size() && this->required_bool_num == td.required_bools.size() &&
           this->required_string_num == td.required_strings.size();
}
}  // namespace luhsoccer::local_planner