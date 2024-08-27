#include "robot_control/components/component_util.hpp"
#include "robot_control/components/component_data.hpp"
#include "config_provider/config_store_main.hpp"
#include "core/visit.hpp"
namespace luhsoccer::robot_control {

const config_provider::RobotControlConfig& robotControlConfig() {
    return config_provider::ConfigProvider::getConfigStore().robot_control_config;
}

const config_provider::RobotControlVisualizationConfig& robotControlVisualizationConfig() {
    return config_provider::ConfigProvider::getConfigStore().robot_control_visualization_config;
}

// ----------------------------------------------- Bool ----------------------------------------------------------------
template <>
ComponentParam<config_provider::BoolParamClass, bool>::ComponentParam(CallbackDefine,
                                                                      const std::function<bool(CallbackData)>& callback)
    : callback2([callback](const ComponentData& data, const ComponentUid& uid) {
          return callback({data.wm, data.td, uid});
      }){};

// template <>
// ComponentParam<config_provider::BoolParamClass, bool>::ComponentParam(CallbackDefine,
//                                                                       const SkillCallback<bool>& callback)
//     : callback2([callback](const ComponentData& data, const ComponentUid&) { return callback(data.wm, data.td); }){};

template <>
ComponentParam<config_provider::BoolParamClass, bool>::ComponentParam(TaskDataDefine, int i)
    : callback2([i](const ComponentData& data, const ComponentUid&) { return data.td.required_bools[i]; }){};

// ----------------------------------------------- Double --------------------------------------------------------------
template <>
ComponentParam<config_provider::DoubleParamClass, double>::ComponentParam(
    CallbackDefine, const std::function<double(CallbackData)>& callback)
    : callback2([callback](const ComponentData& data, const ComponentUid& uid) {
          return callback({data.wm, data.td, uid});
      }){};

// template <>
// ComponentParam<config_provider::DoubleParamClass, double>::ComponentParam(
//     CallbackDefine,
//     const std::function<double(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td)>&
//     callback) : callback2([callback](const ComponentData& data, const ComponentUid&) { return callback(data.wm,
//     data.td); }){};

template <>
ComponentParam<config_provider::DoubleParamClass, double>::ComponentParam(TaskDataDefine, int i)
    : callback2([i](const ComponentData& data, const ComponentUid&) { return data.td.required_doubles[i]; }){};

// ----------------------------------------------- Int --------------------------------------------------------------
template <>
ComponentParam<config_provider::IntParamClass, int>::ComponentParam(CallbackDefine,
                                                                    const std::function<int(CallbackData)>& callback)
    : callback2([callback](const ComponentData& data, const ComponentUid& uid) {
          return callback({data.wm, data.td, uid});
      }){};

// template <>
// ComponentParam<config_provider::IntParamClass, int>::ComponentParam(CallbackDefine, const SkillCallback<int>&
// callback)
//     : callback2([callback](const ComponentData& data, const ComponentUid&) { return callback(data.wm, data.td); }){};

template <>
ComponentParam<config_provider::IntParamClass, int>::ComponentParam(TaskDataDefine, int i)
    : callback2([i](const ComponentData& data, const ComponentUid&) { return data.td.required_ints[i]; }){};

// ----------------------------------------------- Int --------------------------------------------------------------
template <>
ComponentParam<config_provider::StringParamClass, std::string>::ComponentParam(
    CallbackDefine, const std::function<std::string(CallbackData)>& callback)
    : callback2([callback](const ComponentData& data, const ComponentUid& uid) {
          return callback({data.wm, data.td, uid});
      }){};

// template <>
// ComponentParam<config_provider::StringParamClass, std::string>::ComponentParam(
//     CallbackDefine, const SkillCallback<std::string>& callback)
//     : callback2([callback](const ComponentData& data, const ComponentUid&) { return callback(data.wm, data.td); }){};

template <>
ComponentParam<config_provider::StringParamClass, std::string>::ComponentParam(TaskDataDefine, int i)
    : callback2([i](const ComponentData& data, const ComponentUid&) { return data.td.required_strings[i]; }){};

transform::Position ComponentPosition::positionObject(const ComponentData& comp_data) const {
    if (this->position) {
        return this->position.value();
    } else if (this->task_data_reference) {
        switch (this->task_data_reference->first) {
            case TaskDataType::ROBOT:
                if (comp_data.td.related_robots.size() > this->task_data_reference->second) {
                    return {comp_data.td.related_robots[this->task_data_reference->second].getFrame()};
                } else {
                    throw std::runtime_error("Index for related robots vector of task data is to high!");
                }
                break;
            case TaskDataType::POINT:
                if (comp_data.td.required_positions.size() > this->task_data_reference->second) {
                    return comp_data.td.required_positions[this->task_data_reference->second];
                } else {
                    throw std::runtime_error("Index for required_point vector of task data is to high!");
                }
                break;
            case TaskDataType::EXECUTING_ROBOT:
                return {comp_data.td.robot.getFrame()};
            default:
                throw std::runtime_error("Unhandled enum provided!");
                break;
        }
    } else {
        auto visitor = overload{
            [&comp_data](const SkillCallback<transform::Position>& c) { return c(comp_data.wm, comp_data.td); },
            [](const std::function<transform::Position()>& c) { return c(); },
            [&comp_data, this](const std::function<transform::Position(CallbackData)>& c) -> transform::Position {
                return c({comp_data.wm, comp_data.td, this->getComponentUid()});
            },
            [&comp_data, this](const std::function<transform::Position(const ComponentData&, const ComponentUid&)>& c) {
                return c(comp_data, this->getComponentUid());
            },
            [this](const NonCallback&) {
                throw std::runtime_error(
                    fmt::format("No SkillPositionType defined in ComponentPosition with id {:d}!", this->getUid()));
                return transform::Position("");  // this in only so the visit method can be used
            }};
        return std::visit(visitor, this->callback);
    }
}

ComponentPosition::ComponentPosition(const transform::Position& position)
    : position(position), callback(NON_CALLBACK){};

ComponentPosition::ComponentPosition(const char* frame) : position(frame), callback(NON_CALLBACK) {}

ComponentPosition::ComponentPosition(const RobotIdentifier& robot)
    : position(robot.getFrame()), callback(NON_CALLBACK){};

ComponentPosition::ComponentPosition(const TaskDataType type, const size_t i)
    : task_data_reference({type, i}), callback(NON_CALLBACK){};

ComponentPosition::ComponentPosition(CallbackDefine, const SkillCallback<transform::Position>& callback)
    : callback(callback){};
ComponentPosition::ComponentPosition(CallbackDefine, const std::function<transform::Position()>& callback)
    : callback(callback){};
ComponentPosition::ComponentPosition(CallbackDefine, const std::function<transform::Position(CallbackData)>& callback)
    : callback(callback){};
ComponentPosition::ComponentPosition(
    CallbackDefine, const std::function<transform::Position(const ComponentData&, const ComponentUid&)>& callback)
    : callback(callback){};
}  // namespace luhsoccer::robot_control