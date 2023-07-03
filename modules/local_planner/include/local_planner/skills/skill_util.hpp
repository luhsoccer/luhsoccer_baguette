
#pragma once

#include <memory>
#include <optional>
#include <variant>

#include "transform/position.hpp"
#include "local_planner/local_planner_util.hpp"
#include "config_provider/config_store_main.hpp"
#include "local_planner/skills/abstract_component.hpp"

#include "visit.hpp"
namespace luhsoccer::local_planner {

struct TaskData;

inline const config_provider::LocalPlannerComponentsConfig& localPlannerConfig() {
    return config_provider::ConfigProvider::getConfigStore().local_planner_components_config;
}

/**
 * @brief A callback that can be given to calculate a position or parameter
 *
 * @tparam T the type that should be returned by the callback
 */
template <typename T>
using SkillCallback = std::function<T(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td)>;

/**
 * @brief A param of a local planner component
 *
 * @tparam P the param config class
 * @tparam T the underling type
 */

class CallbackDefine {};
constexpr CallbackDefine CALLBACK;

class TaskDataDefine {};
constexpr TaskDataDefine TD;

class NonCallback {};
constexpr NonCallback NON_CALLBACK;

struct CallbackData {
    const std::shared_ptr<const transform::WorldModel>& wm;
    const TaskData& td;
    const ComponentUid component_uid;
};

template <class P, typename T>
class ComponentParam : private LocalPlannerParam<P, T>, AbstractComponent {
   public:
    /**
     * @brief Define a component param by a constant value
     *
     * @param constant constant value
     */
    ComponentParam(const T constant) : LocalPlannerParam<P, T>(constant), callback(NON_CALLBACK){};

    /**
     * @brief Define a component param by a config param
     *
     * @param param reference of the param from the config
     */
    ComponentParam(const P& param) : LocalPlannerParam<P, T>(param), callback(NON_CALLBACK){};

    /**
     * @brief Define a component param by a function
     *
     * @param callback callback that should be called to calculated the param
     */
    ComponentParam(CallbackDefine, const SkillCallback<T>& callback) : LocalPlannerParam<P, T>(), callback(callback){};
    ComponentParam(CallbackDefine, const std::function<T()>& callback)
        : LocalPlannerParam<P, T>(), callback(callback){};
    ComponentParam(CallbackDefine, const std::function<T(CallbackData)>& callback)
        : LocalPlannerParam<P, T>(), callback(callback){};

    /**
     * @brief Define a component param by the task data
     *
     * @param i index in the task data
     */
    ComponentParam(TaskDataDefine, int i);

    ~ComponentParam() override = default;
    ComponentParam(const ComponentParam&) = default;
    ComponentParam& operator=(const ComponentParam&) = default;
    ComponentParam(ComponentParam&&) noexcept = default;
    ComponentParam& operator=(ComponentParam&&) noexcept = default;

    /**
     * @brief get the value of this param
     *
     * @param wm WorldModel
     * @param td TaskData
     * @return T the value of this param
     */
    [[nodiscard]] T val(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) const {
        auto visitor = overload{[&wm, &td](const SkillCallback<T>& c) { return c(wm, td); },
                                [](const std::function<T()>& c) { return c(); },
                                [&wm, &td, this](const std::function<T(CallbackData)>& c) {
                                    return c({wm, td, this->getComponentUid()});
                                },
                                [this](const NonCallback&) { return LocalPlannerParam<P, T>::val(); }};
        return std::visit(visitor, this->callback);
    }

   private:
    std::variant<SkillCallback<T>, std::function<T()>, std::function<T(CallbackData)>, NonCallback> callback;
};

using BoolComponentParam = ComponentParam<config_provider::BoolParamClass, bool>;
using DoubleComponentParam = ComponentParam<config_provider::DoubleParamClass, double>;
using IntComponentParam = ComponentParam<config_provider::IntParamClass, int>;
using StringComponentParam = ComponentParam<config_provider::StringParamClass, std::string>;

/**
 * @brief A position parameter of a local planner component.
 * e.g: target position
 */
class ComponentPosition : AbstractComponent {
   public:
    /**
     * @brief whether a related robot or a required point is meant, if the component position is defined in the task
     * data
     *
     */
    enum class TaskDataType { ROBOT, POINT, EXECUTING_ROBOT };

    /**
     * @brief Define the component position by a (constant) transform position
     *
     * @param position transform position
     */
    ComponentPosition(const transform::Position& position) : position(position), callback(NON_CALLBACK){};

    /**
     * @brief Define the component position by a frame name
     *
     * @param frame const char* name of the frame
     */
    ComponentPosition(const char* frame) : position(frame), callback(NON_CALLBACK) {}

    /**
     * @brief Define the component position by a robot identifier
     *
     * @param robot RobotIdentifier the robot
     */
    ComponentPosition(const RobotIdentifier& robot) : position(robot.getFrame()), callback(NON_CALLBACK){};

    /**
     * @brief Define the component position by the task data
     *
     * @param type whether a related robot or a required point is meant
     * @param i the index of the related robot array/required point array. If @p type is EXECUTING_ROBOT this parameter
     * is ignored.
     */
    ComponentPosition(const TaskDataType type, const size_t i)
        : task_data_reference({type, i}), callback(NON_CALLBACK){};

    /**
     * @brief Define the component position by a callback
     *
     * @param cs reference to the config store
     * @param callback the callback that should be called
     */
    ComponentPosition(CallbackDefine, const SkillCallback<transform::Position>& callback) : callback(callback){};
    ComponentPosition(CallbackDefine, const std::function<transform::Position()>& callback) : callback(callback){};
    ComponentPosition(CallbackDefine, const std::function<transform::Position(CallbackData)>& callback)
        : callback(callback){};

    /**
     * @brief get the current position object
     *
     * @param wm WorldModel
     * @param td TaskData
     * @return transform::Position position object
     */
    [[nodiscard]] transform::Position positionObject(const std::shared_ptr<const transform::WorldModel>& wm,
                                                     const TaskData& td) const;

   private:
    std::optional<transform::Position> position;
    std::optional<std::pair<TaskDataType, size_t>> task_data_reference;
    std::variant<SkillCallback<transform::Position>, std::function<transform::Position()>,
                 std::function<transform::Position(CallbackData)>, NonCallback>
        callback;
};

}  // namespace luhsoccer::local_planner