

#pragma once

#include "config/robot_control_config.hpp"
#include "robot_control/components/component_util.hpp"
#include "robot_control/components/abstract_component.hpp"

namespace luhsoccer::robot_control {

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
    [[nodiscard]] virtual std::pair<double, bool> calcRotationVelocity(
        const ComponentData& comp_data, const transform::Position& robot_position) const = 0;

   protected:
    /**
     * @brief Construct a new Abstract Rotation Control object
     *
     * @param max_vel_theta optional custom absolute maximum rotational velocity
     */
    AbstractRotationControl(const std::optional<DoubleComponentParam>& max_vel_theta = std::nullopt)
        : max_vel_theta(max_vel_theta.has_value() ? max_vel_theta.value() : robotControlConfig().robot_max_vel_theta){};

    /**
     * @brief limit a rotational velocity to the maximum absolute rotational velocity
     *
     * @param wm
     * @param td
     * @param rot_vel
     * @return double
     */
    [[nodiscard]] double limitVelocityVector(const ComponentData& comp_data, double rot_vel) const {
        rot_vel *= std::min(1.0, max_vel_theta.val(comp_data) / std::abs(rot_vel));
        return rot_vel;
    };

    /// @brief maximum absolute rotational velocity [rad/s].
    DoubleComponentParam max_vel_theta;
};
}  // namespace luhsoccer::robot_control