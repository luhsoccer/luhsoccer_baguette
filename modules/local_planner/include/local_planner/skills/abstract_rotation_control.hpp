

#pragma once

#include "local_planner/skills/skill_util.hpp"
#include "marker_service/marker.hpp"
#include "local_planner/skills/abstract_component.hpp"

namespace luhsoccer::local_planner {

/**
 * @brief Implements a base class to control the heading of a robot during a step
 *
 */
class AbstractRotationControl : public AbstractComponent {
   public:
    ~AbstractRotationControl() override = default;
    AbstractRotationControl(const AbstractRotationControl&) = default;
    AbstractRotationControl& operator=(const AbstractRotationControl&) = default;
    AbstractRotationControl(AbstractRotationControl&&) = default;
    AbstractRotationControl& operator=(AbstractRotationControl&&) = default;

    /**
     * @brief calculate the momentum that should be applied on the robot
     *
     * @param wm WorldModel
     * @param td TaskData
     * @param robot robot
     * @param time time point
     * @return std::pair<double, bool> momentum that should be applied on the robot, whether the final target heading is
     * reached
     */
    [[nodiscard]] virtual std::pair<double, bool> calcRotationForce(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const time::TimePoint& time = time::TimePoint(0)) const = 0;

    /**
     * @brief Get the current Visualization Marker of the robot rotation control
     *
     * @param wm WorldModel
     * @param td TaskData
     * @param robot robot
     * @param time time point
     * @return Marker Visualization Marker
     */
    [[nodiscard]] virtual std::vector<Marker> getVisualizationMarker(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        time::TimePoint time = time::TimePoint(0)) const = 0;

   protected:
    /**
     * @brief Construct a new Abstract Rotation Control object
     *
     * @param max_vel_theta optional custom absolute maximum rotational velocity
     */
    AbstractRotationControl(const std::optional<DoubleComponentParam>& max_vel_theta = std::nullopt)
        : max_vel_theta(localPlannerConfig().robot_vel_max_theta) {
        if (max_vel_theta) {
            this->max_vel_theta = *max_vel_theta;
        }
    };

    /**
     * @brief limit a rotational velocity to the maximum absolute rotational velocity
     *
     * @param wm
     * @param td
     * @param rot_vel
     * @return double
     */
    [[nodiscard]] double limitVelocityVector(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                                             double rot_vel) const {
        rot_vel *= std::min(1.0, max_vel_theta.val(wm, td) / std::abs(rot_vel));
        return rot_vel;
    };

    /**
     * @brief convert an angle to the shorter one, if its over PI.
     * e.g: 1.5 * pi -> -0.5 * pi
     */
    [[nodiscard]] static double cropAngle(double angle) {
        if (angle > L_PI) {
            angle -= 2 * L_PI;
        }
        if (angle < -L_PI) {
            angle += 2 * L_PI;
        }
        return angle;
    }

    /// @brief maximum absolute rotational velocity [rad/s].
    DoubleComponentParam max_vel_theta;
};
}  // namespace luhsoccer::local_planner