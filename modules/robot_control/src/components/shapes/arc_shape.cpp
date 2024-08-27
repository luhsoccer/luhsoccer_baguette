#include "robot_control/components/shapes/arc_shape.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include "robot_control/components/component_data.hpp"
#include "marker_adapter.hpp"
#include "marker_service/marker.hpp"
#include "utils/utils.hpp"
namespace luhsoccer::robot_control {

VectorWithVelocityStamped ArcShape::getTransformToClosestPoint(const ComponentData& comp_data,
                                                               const transform::Position& robot_position,
                                                               const transform::Position& observe_position) const {
    VectorWithVelocityStamped trans_and_vel;
    trans_and_vel.stamp = comp_data.time;
    // translation
    std::optional<Eigen::Affine2d> translation_to_center_in_obs_frame =
        this->center.positionObject(comp_data).getCurrentPosition(comp_data.wm, observe_position, comp_data.time);
    std::optional<Eigen::Affine2d> translation_to_ref_in_obs_frame =
        robot_position.getCurrentPosition(comp_data.wm, observe_position, comp_data.time);
    if (translation_to_ref_in_obs_frame && translation_to_center_in_obs_frame) {
        auto start = defineStart(comp_data);
        auto end = defineEnd(comp_data);
        if (start && end) {
            double start_angle = start->first;
            Eigen::Vector2d start_point = start->second;

            double end_angle = end->first;
            Eigen::Vector2d end_point = end->second;

            Eigen::Vector2d reference_to_center_in_obs_frame =
                translation_to_center_in_obs_frame->translation() - translation_to_ref_in_obs_frame->translation();
            double reference_angle =
                std::atan2(-reference_to_center_in_obs_frame.y(), -reference_to_center_in_obs_frame.x());
            double angle_from_start = reference_angle - start_angle;
            if (angle_from_start < 0.0) angle_from_start += 2 * L_PI;
            if (angle_from_start < local_angle.val(comp_data)) {
                // vector to the closest point on the arc
                trans_and_vel.vec = reference_to_center_in_obs_frame *
                                    (1 - radius.val(comp_data) / reference_to_center_in_obs_frame.norm());
            } else if (std::abs(cropAngle(reference_angle - start_angle)) <
                       std::abs(cropAngle(reference_angle - end_angle))) {
                // vector to the start point
                trans_and_vel.vec = start_point - translation_to_ref_in_obs_frame->translation();
            } else {
                // vector to the end point
                trans_and_vel.vec = end_point - translation_to_ref_in_obs_frame->translation();
            }
        }
    }

    // velocity
    std::optional<Eigen::Vector3d> velocity = this->center.positionObject(comp_data).getVelocity(
        comp_data.wm, robot_position.getFrame(), observe_position.getFrame(), comp_data.time);
    if (velocity) {
        trans_and_vel.velocity = velocity->head(2);
    }

    // visualization
    marker::LineStrip arc_line(center.positionObject(comp_data));
    std::vector<marker::Point> points;
    constexpr size_t FACES = 32;
    points.reserve(FACES);
    for (size_t i = 0; i < FACES; i++) {
        double angle = -local_angle.val(comp_data) / 2 +
                       static_cast<double>(i) * local_angle.val(comp_data) / (FACES - 1) + L_PI / 2;
        double s_point_x = std::sin(angle) * radius.val(comp_data);
        double s_point_y = std::cos(angle) * radius.val(comp_data);
        points.emplace_back(s_point_x, s_point_y);
    }
    arc_line.setPoints(points);
    comp_data.ma.displayMarker(arc_line);

    return trans_and_vel;
}

std::optional<Eigen::Vector2d> ArcShape::getCenter(const ComponentData& comp_data,
                                                   const transform::Position& observe_position) const {
    std::optional<Eigen::Affine2d> translation_to_center_in_obs_frame =
        center.positionObject(comp_data).getCurrentPosition(comp_data.wm, observe_position, comp_data.time);
    if (translation_to_center_in_obs_frame.has_value()) {
        double global_angle = Eigen::Rotation2Dd(translation_to_center_in_obs_frame->rotation()).angle();
        double x_middle =
            translation_to_center_in_obs_frame->translation().x() + std::cos(global_angle) * radius.val(comp_data);
        double y_middle =
            translation_to_center_in_obs_frame->translation().y() + std::sin(global_angle) * radius.val(comp_data);

        return Eigen::Vector2d{x_middle, y_middle};
    }
    return std::nullopt;
}

std::optional<std::pair<double, Eigen::Vector2d>> ArcShape::defineStart(
    const ComponentData& comp_data, const transform::Position& observe_position) const {
    std::optional<Eigen::Affine2d> translation_to_center_in_obs_frame =
        center.positionObject(comp_data).getCurrentPosition(comp_data.wm, observe_position, comp_data.time);
    if (translation_to_center_in_obs_frame) {
        // define angle
        double global_angle = Eigen::Rotation2Dd(translation_to_center_in_obs_frame->rotation()).angle();
        double start_angle = cropAngle(global_angle - local_angle.val(comp_data) / 2);
        // define coordinates
        double x_start =
            translation_to_center_in_obs_frame->translation().x() + std::cos(start_angle) * radius.val(comp_data);
        double y_start =
            translation_to_center_in_obs_frame->translation().y() + std::sin(start_angle) * radius.val(comp_data);
        return std::pair<double, Eigen::Vector2d>{start_angle, Eigen::Vector2d(x_start, y_start)};
    }
    return std::nullopt;  // error
}
std::optional<std::pair<double, Eigen::Vector2d>> ArcShape::defineEnd(
    const ComponentData& comp_data, const transform::Position& observe_position) const {
    std::optional<Eigen::Affine2d> translation_to_center_in_obs_frame =
        center.positionObject(comp_data).getCurrentPosition(comp_data.wm, observe_position, comp_data.time);
    if (translation_to_center_in_obs_frame) {
        // define angle
        double global_angle = Eigen::Rotation2Dd(translation_to_center_in_obs_frame->rotation()).angle();
        double end_angle = cropAngle(global_angle + local_angle.val(comp_data) / 2);
        // define coordinates
        double x_end =
            translation_to_center_in_obs_frame->translation().x() + std::cos(end_angle) * radius.val(comp_data);
        double y_end =
            translation_to_center_in_obs_frame->translation().y() + std::sin(end_angle) * radius.val(comp_data);

        return std::pair<double, Eigen::Vector2d>{end_angle, Eigen::Vector2d(x_end, y_end)};
    }
    return std::nullopt;  // error
}
}  // namespace luhsoccer::robot_control