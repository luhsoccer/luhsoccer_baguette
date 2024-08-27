#pragma once

#include "robot_interface/robot_interface_types.hpp"
#include "robot_control/components/abstract_component.hpp"

namespace luhsoccer {

namespace transform {
class WorldModel;
}
namespace robot_control {
struct ComponentData;
/**
 * @brief A base step in a skill.
 *
 */
class AbstractStep : public AbstractComponent {
   public:
    ~AbstractStep() override = default;
    AbstractStep(const AbstractStep&) = default;
    AbstractStep& operator=(const AbstractStep&) = default;
    AbstractStep(AbstractStep&&) = default;
    AbstractStep& operator=(AbstractStep&&) = default;
    /**
     * @brief Current state of this step
     *
     */
    enum class StepState {
        RUNNING,   ///< the step is in the normal operational state
        FINISHED,  ///< the step is finished and the next step should be executed
        ABORT,     ///< the step was aborted from outside
        ERROR      ///< an error occurred
    };

    /**
     * @brief Calculate the current command Message for the robot
     *
     * @param wm WorldModel
     * @param td TaskData
     * @param robot robot
     * @param time time point
     * @return std::pair<StepState, robot_interface::RobotCommand> the current state of this step, the current
     * command message
     */
    [[nodiscard]] virtual std::pair<StepState, robot_interface::RobotCommand> calcCommandMessage(
        const ComponentData& comp_data) const = 0;

   protected:
    AbstractStep() = default;
};
}  // namespace robot_control
}  // namespace luhsoccer