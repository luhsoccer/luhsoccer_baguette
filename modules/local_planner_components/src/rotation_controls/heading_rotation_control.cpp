
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "logger/logger.hpp"
namespace luhsoccer::local_planner {

std::pair<double, bool> HeadingRotationControl::calcRotationForce(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
    const time::TimePoint& time) const {
    // determine error of heading

    transform::RobotHandle robot_handle(robot, wm);
    double heading_error = 0.0;
    if (this->angle_rotation_target) {
        // calculations if defined by heading angle
        heading_error = this->angle_rotation_target->first.val(wm, td) +
                        Eigen::Rotation2Dd(this->angle_rotation_target->second.positionObject(wm, td)
                                               .getCurrentPosition(wm, robot.getFrame(), time)
                                               ->rotation())
                            .angle();

    } else if (this->position_rotation_target) {
        // calculations if defined by position to align to
        std::optional<Eigen::Affine2d> robot_to_position_transform =
            this->position_rotation_target->first.positionObject(wm, td).getCurrentPosition(wm, robot.getFrame(), time);
        if (robot_to_position_transform) {
            heading_error =
                atan2(robot_to_position_transform->translation().y(), robot_to_position_transform->translation().x());

            // Invert angle if robot should point away
            if (this->position_rotation_target->second.val(wm, td)) {
                heading_error -= L_PI;
            }
        } else {
            return {0, false};
        }
    } else {
        return {0, false};
    }
    heading_error = this->cropAngle(heading_error);
    // LOG_INFO(logger::Logger("HeadingControl"), "heading_error: {:0.4f}", heading_error);
    // determine desired rotational velocity
    double desired_rot_velocity = heading_error * this->rotation_k_g.val(wm, td) / this->rotation_k_v.val(wm, td);

    desired_rot_velocity = this->limitVelocityVector(wm, td, desired_rot_velocity);

    // determine momentum aka. rotational force
    std::optional<Eigen::Vector3d> current_velocity = robot_handle.getPosition().getVelocity(wm, "", "", time);
    if (current_velocity) {
        double rotation_force = this->rotation_k_v.val(wm, td) * (desired_rot_velocity - current_velocity->z());
        bool velocity_zero = true;
        if (this->velocity_zero_for_reach.val(wm, td)) {
            velocity_zero = std::abs(current_velocity->z()) < ZERO_VELOCITY_TOLERANCE;
        }
        return {rotation_force, std::abs(heading_error) < this->heading_tolerance.val(wm, td) && velocity_zero};
    } else {
        return {0, false};
    }
}

std::vector<Marker> HeadingRotationControl::getVisualizationMarker(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
    time::TimePoint time) const {
    constexpr double LINE_LENGTH = 0.15;
    transform::RobotHandle robot_handle(robot, wm);
    if (this->angle_rotation_target) {
        // set marker position to robot position and add heading offset
        marker::Line m(
            transform::Position(robot.getFrame(), 0, 0,
                                this->angle_rotation_target->first.val(wm, td) +
                                    Eigen::Rotation2Dd(this->angle_rotation_target->second.positionObject(wm, td)
                                                           .getCurrentPosition(wm, robot_handle.getPosition(), time)
                                                           ->rotation())
                                        .angle()));
        m.setLinePoints({0.0, 0.0}, {LINE_LENGTH, 0.0});
        m.setColor(marker::Color::GREEN());
        return {m};
    } else if (this->position_rotation_target) {
        marker::Line m("");
        auto robot_postion = robot_handle.getPosition().getCurrentPosition(wm, "", time);
        marker::Point start_point;
        if (robot_postion) {
            start_point.x = robot_postion->translation().x();
            start_point.y = robot_postion->translation().y();
        }
        marker::Point end_point;
        auto align_pose_position =
            position_rotation_target->first.positionObject(wm, td).getCurrentPosition(wm, "", time);
        if (align_pose_position && robot_postion) {
            Eigen::Vector2d direction_vec = {align_pose_position->translation().x() - robot_postion->translation().x(),
                                             align_pose_position->translation().y() - robot_postion->translation().y()};
            direction_vec.normalize();
            end_point = start_point;
            end_point.x += direction_vec.x() * LINE_LENGTH;
            end_point.y += direction_vec.y() * LINE_LENGTH;
        }
        m.setLinePoints(start_point, end_point);
        m.setColor(marker::Color::GREEN());
        return {m};
    } else {
        return {marker::Line("")};
    }
}

}  // namespace luhsoccer::local_planner