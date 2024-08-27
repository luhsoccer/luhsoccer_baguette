
#pragma once

#include <memory>
#include <optional>
#include <variant>

#include "config_provider/parameters.hpp"
#include "marker_service/marker.hpp"
#include "transform/position.hpp"
#include "robot_control/components/abstract_component.hpp"

namespace luhsoccer::config_provider {
struct RobotControlConfig;
struct RobotControlVisualizationConfig;
}  // namespace luhsoccer::config_provider

namespace luhsoccer::robot_control {

const config_provider::RobotControlConfig& robotControlConfig();
const config_provider::RobotControlVisualizationConfig& robotControlVisualizationConfig();

constexpr marker::Color RC_GREEN{125, 249, 255};
constexpr marker::Color RC_RED{155, 48, 255};
constexpr marker::Color RC_ORANGE{255, 211, 0};

struct ComponentData;

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
class ComponentParam : AbstractComponent {
   public:
    /**
     * @brief Define a component param by a constant value
     *
     * @param constant constant value
     */
    ComponentParam(const T& constant) : constant(constant){};

    /**
     * @brief Define a component param by a config param
     *
     * @param param reference of the param from the config
     */
    ComponentParam(const P& param) : param(&param){};

    /**
     * @brief Define a component param by a function
     *
     * @param callback callback that should be called to calculated the param
     */
    // ComponentParam(CallbackDefine, const SkillCallback<T>& callback);
    ComponentParam(CallbackDefine, const std::function<T()>& callback)
        : callback2([callback](const ComponentData&, const ComponentUid&) { return callback(); }){};
    ComponentParam(CallbackDefine, const std::function<T(CallbackData)>& callback);
    ComponentParam(CallbackDefine, const std::function<T(const ComponentData&, const ComponentUid&)>& callback)
        : callback2(callback){};

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
    [[nodiscard]] T val(const ComponentData& comp_data) const {
        if (this->constant.has_value()) return this->constant.value();
        if (this->param != nullptr) return this->param->val();
        if (this->callback2.has_value()) return this->callback2.value()(comp_data, this->getComponentUid());
        throw std::runtime_error("No ComponentParam option defined!");
    }

   private:
    const P* param{nullptr};
    std::optional<T> constant;
    std::optional<std::function<T(const ComponentData&, const ComponentUid&)>> callback2;
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
    ComponentPosition(const transform::Position& position);

    /**
     * @brief Define the component position by a frame name
     *
     * @param frame const char* name of the frame
     */
    ComponentPosition(const char* frame);

    /**
     * @brief Define the component position by a robot identifier
     *
     * @param robot RobotIdentifier the robot
     */
    ComponentPosition(const RobotIdentifier& robot);

    /**
     * @brief Define the component position by the task data
     *
     * @param type whether a related robot or a required point is meant
     * @param i the index of the related robot array/required point array. If @p type is EXECUTING_ROBOT this parameter
     * is ignored.
     */
    ComponentPosition(const TaskDataType type, const size_t i);

    /**
     * @brief Define the component position by a callback
     *
     * @param cs reference to the config store
     * @param callback the callback that should be called
     */
    ComponentPosition(CallbackDefine, const SkillCallback<transform::Position>& callback);
    ComponentPosition(CallbackDefine, const std::function<transform::Position()>& callback);
    ComponentPosition(CallbackDefine, const std::function<transform::Position(CallbackData)>& callback);
    ComponentPosition(CallbackDefine,
                      const std::function<transform::Position(const ComponentData&, const ComponentUid&)>& callback);

    /**
     * @brief get the current position object
     *
     * @param wm WorldModel
     * @param td TaskData
     * @return transform::Position position object
     */
    [[nodiscard]] transform::Position positionObject(const ComponentData& comp_data) const;

   private:
    std::optional<transform::Position> position;
    std::optional<std::pair<TaskDataType, size_t>> task_data_reference;
    std::variant<SkillCallback<transform::Position>, std::function<transform::Position()>,
                 std::function<transform::Position(CallbackData)>,
                 std::function<transform::Position(const ComponentData&, const ComponentUid&)>, NonCallback>
        callback;
};

}  // namespace luhsoccer::robot_control