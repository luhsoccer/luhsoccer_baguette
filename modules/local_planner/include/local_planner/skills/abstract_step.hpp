#pragma once

#include <memory>
#include "robot_interface/robot_interface_types.hpp"
#include "marker_service/marker.hpp"
#include "local_planner/skills/abstract_component.hpp"

namespace luhsoccer {

namespace transform {
class WorldModel;
}
namespace local_planner {
class AvoidanceManager;
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
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const std::shared_ptr<AvoidanceManager>& am, const time::TimePoint time = time::TimePoint(0)) const = 0;

    /**
     * @brief Get the current Visualization Markers of this step
     * @param wm WorldModel
     * @param td TaskData
     * @param robot robot
     * @param time time point
     * @return std::vector<Marker> a list of all current visualization markers
     */
    [[nodiscard]] virtual std::vector<Marker> getVisualizationMarkers(
        const std::shared_ptr<const transform::WorldModel>& /*wm*/, const TaskData& /*td*/,
        const RobotIdentifier& /*robot*/, const time::TimePoint /*time*/ = time::TimePoint(0)) const {
        return {};
    };

   protected:
    /**
     * @brief Construct a new Abstract Step object
     *
     */
    AbstractStep() = default;
};
}  // namespace local_planner
}  // namespace luhsoccer