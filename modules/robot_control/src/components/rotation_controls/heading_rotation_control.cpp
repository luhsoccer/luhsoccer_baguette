#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/component_data.hpp"
#include "marker_adapter.hpp"
#include "marker_service/marker.hpp"

namespace luhsoccer::robot_control {

HeadingRotationControl::HeadingRotationControl(const DoubleComponentParam& angle,
                                               const ComponentPosition& observe_position,
                                               DoubleComponentParam heading_tolerance,
                                               BoolComponentParam velocity_zero_for_reach,
                                               const std::optional<DoubleComponentParam>& k_p,
                                               const std::optional<DoubleComponentParam>& max_vel_theta)
    : AbstractRotationControl(max_vel_theta),
      angle_rotation_target({angle, observe_position}),
      heading_tolerance(std::move(heading_tolerance)),
      rotation_k_p(k_p.has_value() ? k_p.value() : robotControlConfig().rotation_k_p),
      velocity_zero_for_reach(std::move(velocity_zero_for_reach)){};

HeadingRotationControl::HeadingRotationControl(const ComponentPosition& align_position,
                                               const BoolComponentParam& face_away,
                                               DoubleComponentParam heading_tolerance,
                                               BoolComponentParam velocity_zero_for_reach,
                                               const std::optional<DoubleComponentParam>& k_p,
                                               const std::optional<DoubleComponentParam>& max_vel_theta)
    : AbstractRotationControl(max_vel_theta),
      position_rotation_target({align_position, face_away}),
      heading_tolerance(std::move(heading_tolerance)),
      rotation_k_p(k_p.has_value() ? k_p.value() : robotControlConfig().rotation_k_p),
      velocity_zero_for_reach(std::move(velocity_zero_for_reach)){};

std::pair<double, bool> HeadingRotationControl::calcRotationVelocity(const ComponentData& comp_data,
                                                                     const transform::Position& robot_position) const {
    constexpr double LINE_LENGTH = 0.15;

    // determine error of heading
    double heading_error = 0.0;
    if (this->angle_rotation_target) {
        // calculations if defined by heading angle
        auto reference_position = this->angle_rotation_target->second.positionObject(comp_data).getCurrentPosition(
            comp_data.wm, robot_position, comp_data.time);
        if (!reference_position.has_value()) return {0, false};
        heading_error = this->angle_rotation_target->first.val(comp_data) +
                        Eigen::Rotation2Dd(reference_position->rotation()).angle();

        // visualization
        //  set marker position to robot position and add heading offset
        marker::Line m(transform::Position(
            robot_position.getFrame(), 0, 0,
            this->angle_rotation_target->first.val(comp_data) +
                Eigen::Rotation2Dd(this->angle_rotation_target->second.positionObject(comp_data)
                                       .getCurrentPosition(comp_data.wm, robot_position.getFrame(), comp_data.time)
                                       ->rotation())
                    .angle()));
        m.setLinePoints({0.0, 0.0}, {LINE_LENGTH, 0.0});
        m.setColor(RC_GREEN);
        comp_data.ma.displayMarker(m);

    } else if (this->position_rotation_target) {
        // calculations if defined by position to align to
        std::optional<Eigen::Affine2d> robot_to_position_transform =
            this->position_rotation_target->first.positionObject(comp_data).getCurrentPosition(
                comp_data.wm, robot_position, comp_data.time);
        if (!robot_to_position_transform.has_value()) return {0, false};

        heading_error =
            atan2(robot_to_position_transform->translation().y(), robot_to_position_transform->translation().x());

        // Invert angle if robot should point away
        if (this->position_rotation_target->second.val(comp_data)) {
            heading_error -= L_PI;
        }

        // visualization
        marker::Line m("");
        auto robot_postion = robot_position.getCurrentPosition(comp_data.wm, "", comp_data.time);
        marker::Point start_point;
        if (robot_postion) {
            start_point.x = robot_postion->translation().x();
            start_point.y = robot_postion->translation().y();
        }
        marker::Point end_point;
        auto align_pose_position = position_rotation_target->first.positionObject(comp_data).getCurrentPosition(
            comp_data.wm, "", comp_data.time);
        if (align_pose_position && robot_postion) {
            Eigen::Vector2d direction_vec = {align_pose_position->translation().x() - robot_postion->translation().x(),
                                             align_pose_position->translation().y() - robot_postion->translation().y()};
            direction_vec.normalize();
            end_point = start_point;
            end_point.x += direction_vec.x() * LINE_LENGTH;
            end_point.y += direction_vec.y() * LINE_LENGTH;
        }
        m.setLinePoints(start_point, end_point);
        m.setColor(RC_GREEN);
        comp_data.ma.displayMarker(m);

    } else {
        return {0, false};
    }
    heading_error = cropAngle(heading_error);
    // LOG_INFO(logger::Logger("HeadingControl"), "heading_error: {:0.4f}", heading_error);
    // determine desired rotational velocity
    double desired_rot_velocity = heading_error * this->rotation_k_p.val(comp_data);

    desired_rot_velocity = this->limitVelocityVector(comp_data, desired_rot_velocity);

    // determine momentum aka. rotational force
    std::optional<Eigen::Vector3d> current_velocity = robot_position.getVelocity(comp_data.wm, "", "", comp_data.time);
    if (current_velocity) {
        bool velocity_zero = true;
        if (this->velocity_zero_for_reach.val(comp_data)) {
            velocity_zero = std::abs(current_velocity->z()) < ZERO_VELOCITY_TOLERANCE;
        }
        return {desired_rot_velocity,
                std::abs(heading_error) < this->heading_tolerance.val(comp_data) && velocity_zero};
    } else {
        return {0, false};
    }
}
}  // namespace luhsoccer::robot_control