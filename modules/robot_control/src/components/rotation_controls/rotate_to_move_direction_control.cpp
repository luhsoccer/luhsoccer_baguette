#include "robot_control/components/rotation_controls/rotate_to_move_direction_control.hpp"
#include "robot_control/components/component_data.hpp"
#include "marker_adapter.hpp"
#include "marker_service/marker.hpp"

namespace luhsoccer::robot_control {

std::pair<double, bool> RotateToMoveDirectionControl::calcRotationVelocity(
    const ComponentData& comp_data, const transform::Position& robot_position) const {
    // determine error of heading

    auto velocity = robot_position.getVelocity(comp_data.wm, "", robot_position, comp_data.time);
    if (!velocity.has_value()) {
        return {0, false};
    }
    double heading_error = std::atan2(velocity->y() / robotControlConfig().robot_max_vel_x,
                                      velocity->x() / robotControlConfig().robot_max_vel_x);
    if (reverse_direction.val(comp_data)) heading_error -= L_PI;
    heading_error = cropAngle(heading_error);
    // LOG_INFO(logger::Logger("HeadingControl"), "heading_error: {:0.4f}", heading_error);
    // determine desired rotational velocity
    double desired_rot_velocity = heading_error * this->k_p.val(comp_data);

    desired_rot_velocity = this->limitVelocityVector(comp_data, desired_rot_velocity);

    // visualization
    constexpr double LINE_LENGTH = 0.15;

    // set marker position to robot position and add heading offset
    marker::Line m(robot_position);
    m.setLinePoints({0.0, 0.0}, {velocity->normalized().x() * LINE_LENGTH, velocity->normalized().y() * LINE_LENGTH});
    m.setColor(RC_GREEN);
    comp_data.ma.displayMarker(m);

    return {desired_rot_velocity, true};
}

}  // namespace luhsoccer::robot_control