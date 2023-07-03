
#include "local_planner_components/rotation_controls/rotate_to_move_direction_control.hpp"
#include "logger/logger.hpp"
namespace luhsoccer::local_planner {

std::pair<double, bool> RotateToMoveDirectionControl::calcRotationForce(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
    const time::TimePoint& time) const {
    // determine error of heading

    transform::RobotHandle robot_handle(robot, wm);
    auto velocity = wm->getVelocity(robot.getFrame(), "", robot.getFrame(), time);
    if (!velocity.has_value()) {
        return {0, false};
    }
    double heading_error = std::atan2(velocity->velocity.y() / localPlannerConfig().robot_vel_max_x,
                                      velocity->velocity.x() / localPlannerConfig().robot_vel_max_x);
    if (reverse_direction.val(wm, td)) heading_error -= L_PI;
    heading_error = this->cropAngle(heading_error);
    // LOG_INFO(logger::Logger("HeadingControl"), "heading_error: {:0.4f}", heading_error);
    // determine desired rotational velocity
    double desired_rot_velocity = heading_error * this->rotation_k_g.val(wm, td) / this->rotation_k_v.val(wm, td);

    desired_rot_velocity = this->limitVelocityVector(wm, td, desired_rot_velocity);

    // determine momentum aka. rotational force
    std::optional<Eigen::Vector3d> current_velocity = robot_handle.getPosition().getVelocity(wm, "", "", time);
    if (current_velocity) {
        double rotation_force = this->rotation_k_v.val(wm, td) * (desired_rot_velocity - current_velocity->z());

        return {rotation_force, true};
    } else {
        return {0, false};
    }
}

std::vector<Marker> RotateToMoveDirectionControl::getVisualizationMarker(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/, const RobotIdentifier& robot,
    time::TimePoint time) const {
    constexpr double LINE_LENGTH = 0.15;
    transform::RobotHandle robot_handle(robot, wm);

    auto velocity = wm->getVelocity(robot.getFrame(), "", robot.getFrame(), time);
    if (!velocity.has_value()) return {marker::Line("")};

    // set marker position to robot position and add heading offset
    marker::Line m(transform::Position(robot.getFrame(), 0, 0, 0));
    m.setLinePoints({0.0, 0.0}, {velocity->velocity.normalized().x() * LINE_LENGTH,
                                 velocity->velocity.normalized().y() * LINE_LENGTH});
    m.setColor(marker::Color::GREEN());
    return {m};
}

}  // namespace luhsoccer::local_planner