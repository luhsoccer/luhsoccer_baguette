#include "local_planner/skills/skill_util.hpp"

#include "local_planner/skills/task.hpp"

namespace luhsoccer::local_planner {
template <>
ComponentParam<config_provider::BoolParamClass, bool>::ComponentParam(TaskDataDefine, int i)
    : callback([i](const std::shared_ptr<const transform::WorldModel>&, const TaskData& td) {
          return td.required_bools[i];
      }){};

template <>
ComponentParam<config_provider::DoubleParamClass, double>::ComponentParam(TaskDataDefine, int i)
    : callback([i](const std::shared_ptr<const transform::WorldModel>&, const TaskData& td) {
          return td.required_doubles[i];
      }){};
template <>
ComponentParam<config_provider::IntParamClass, int>::ComponentParam(TaskDataDefine, int i)
    : callback([i](const std::shared_ptr<const transform::WorldModel>&, const TaskData& td) {
          return td.required_ints[i];
      }){};

template <>
ComponentParam<config_provider::StringParamClass, std::string>::ComponentParam(TaskDataDefine, int i)
    : callback([i](const std::shared_ptr<const transform::WorldModel>&, const TaskData& td) {
          return td.required_strings[i];
      }){};

transform::Position ComponentPosition::positionObject(const std::shared_ptr<const transform::WorldModel>& wm,
                                                      const TaskData& td) const {
    if (this->position) {
        return this->position.value();
    } else if (this->task_data_reference) {
        switch (this->task_data_reference->first) {
            case TaskDataType::ROBOT:
                if (td.related_robots.size() > this->task_data_reference->second) {
                    return {td.related_robots[this->task_data_reference->second].getFrame()};
                } else {
                    throw std::runtime_error("Index for related robots vector of task data is to high!");
                }
                break;
            case TaskDataType::POINT:
                if (td.required_positions.size() > this->task_data_reference->second) {
                    return td.required_positions[this->task_data_reference->second];
                } else {
                    throw std::runtime_error("Index for required_point vector of task data is to high!");
                }
                break;
            case TaskDataType::EXECUTING_ROBOT:
                return {td.robot.getFrame()};
            default:
                throw std::runtime_error("Unhandled enum provided!");
                break;
        }
    } else {
        auto visitor =
            overload{[&wm, &td](const SkillCallback<transform::Position>& c) { return c(wm, td); },
                     [](const std::function<transform::Position()>& c) { return c(); },
                     [&wm, &td, this](const std::function<transform::Position(CallbackData)>& c) {
                         return c({wm, td, this->getComponentUid()});
                     },
                     [this](const NonCallback&) {
                         throw std::runtime_error(fmt::format(
                             "No SkillPositionType defined in ComponentPosition with id {:d}!", this->getUid()));
                         return transform::Position("");  // this in only so the visit method can be used
                     }};
        return std::visit(visitor, this->callback);
    }
}

}  // namespace luhsoccer::local_planner