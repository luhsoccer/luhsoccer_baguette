#include "local_planner_components/shapes/arc_shape.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include "utils/utils.hpp"
namespace luhsoccer::local_planner {

VectorWithVelocityStamped ArcShape::getTransformToClosestPoint(const std::shared_ptr<const transform::WorldModel>& wm,
                                                               const TaskData& td,
                                                               const ComponentPosition& reference_position,
                                                               time::TimePoint time,
                                                               const ComponentPosition& observe_position) const {
    VectorWithVelocityStamped trans_and_vel;
    trans_and_vel.stamp = time;
    // translation
    std::optional<Eigen::Affine2d> translation_to_center_in_obs_frame =
        this->center.positionObject(wm, td).getCurrentPosition(wm, observe_position.positionObject(wm, td), time);
    std::optional<Eigen::Affine2d> translation_to_ref_in_obs_frame =
        reference_position.positionObject(wm, td).getCurrentPosition(wm, observe_position.positionObject(wm, td), time);
    if (translation_to_ref_in_obs_frame && translation_to_center_in_obs_frame) {
        auto start = defineStart(wm, td);
        auto end = defineEnd(wm, td);
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
            if (angle_from_start < local_angle.val(wm, td)) {
                // vector to the closest point on the arc
                trans_and_vel.vec = reference_to_center_in_obs_frame *
                                    (1 - radius.val(wm, td) / reference_to_center_in_obs_frame.norm());
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
    std::optional<Eigen::Vector3d> velocity =
        this->center.positionObject(wm, td).getVelocity(wm, reference_position.positionObject(wm, td).getFrame(),
                                                        observe_position.positionObject(wm, td).getFrame(), time);
    if (velocity) {
        trans_and_vel.velocity = velocity->head(2);
    }
    return trans_and_vel;
}

std::optional<Eigen::Vector2d> ArcShape::getCenter(const std::shared_ptr<const transform::WorldModel>& wm,
                                                   const TaskData& td, time::TimePoint time,
                                                   const ComponentPosition& observe_position) const {
    std::optional<Eigen::Affine2d> translation_to_center_in_obs_frame =
        center.positionObject(wm, td).getCurrentPosition(wm, observe_position.positionObject(wm, td), time);
    if (translation_to_center_in_obs_frame.has_value()) {
        double global_angle = Eigen::Rotation2Dd(translation_to_center_in_obs_frame->rotation()).angle();
        double x_middle =
            translation_to_center_in_obs_frame->translation().x() + std::cos(global_angle) * radius.val(wm, td);
        double y_middle =
            translation_to_center_in_obs_frame->translation().y() + std::sin(global_angle) * radius.val(wm, td);

        return Eigen::Vector2d{x_middle, y_middle};
    }
    return std::nullopt;
}

std::vector<Marker> ArcShape::getVisualizationMarker(const std::shared_ptr<const transform::WorldModel>& wm,
                                                     const TaskData& td, time::TimePoint /*time*/) const {
    marker::LineStrip arc_line(center.positionObject(wm, td));
    std::vector<marker::Point> points;
    constexpr double STEP = 0.052;
    for (double angle = -local_angle.val(wm, td) / 2 + L_PI / 2; angle < local_angle.val(wm, td) / 2 + L_PI / 2;
         angle += STEP) {
        double s_point_x = std::sin(angle) * radius.val(wm, td);
        double s_point_y = std::cos(angle) * radius.val(wm, td);
        points.emplace_back(s_point_x, s_point_y);
    }
    arc_line.setPoints(points);
    return {arc_line};
}

std::optional<std::pair<double, Eigen::Vector2d>> ArcShape::defineStart(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, time::TimePoint time,
    const ComponentPosition& observe_position) const {
    std::optional<Eigen::Affine2d> translation_to_center_in_obs_frame =
        center.positionObject(wm, td).getCurrentPosition(wm, observe_position.positionObject(wm, td), time);
    if (translation_to_center_in_obs_frame) {
        // define angle
        double global_angle = Eigen::Rotation2Dd(translation_to_center_in_obs_frame->rotation()).angle();
        double start_angle = cropAngle(global_angle - local_angle.val(wm, td) / 2);
        // define coordinates
        double x_start =
            translation_to_center_in_obs_frame->translation().x() + std::cos(start_angle) * radius.val(wm, td);
        double y_start =
            translation_to_center_in_obs_frame->translation().y() + std::sin(start_angle) * radius.val(wm, td);
        return std::pair<double, Eigen::Vector2d>{start_angle, Eigen::Vector2d(x_start, y_start)};
    }
    return std::nullopt;  // error
}
std::optional<std::pair<double, Eigen::Vector2d>> ArcShape::defineEnd(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, time::TimePoint time,
    const ComponentPosition& observe_position) const {
    std::optional<Eigen::Affine2d> translation_to_center_in_obs_frame =
        center.positionObject(wm, td).getCurrentPosition(wm, observe_position.positionObject(wm, td), time);
    if (translation_to_center_in_obs_frame) {
        // define angle
        double global_angle = Eigen::Rotation2Dd(translation_to_center_in_obs_frame->rotation()).angle();
        double end_angle = cropAngle(global_angle + local_angle.val(wm, td) / 2);
        // define coordinates
        double x_end = translation_to_center_in_obs_frame->translation().x() + std::cos(end_angle) * radius.val(wm, td);
        double y_end = translation_to_center_in_obs_frame->translation().y() + std::sin(end_angle) * radius.val(wm, td);

        return std::pair<double, Eigen::Vector2d>{end_angle, Eigen::Vector2d(x_end, y_end)};
    }
    return std::nullopt;  // error
}
}  // namespace luhsoccer::local_planner