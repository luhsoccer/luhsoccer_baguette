

#include "local_planner_components/shapes/point_shape.hpp"

namespace luhsoccer::local_planner {

VectorWithVelocityStamped PointShape::getTransformToClosestPoint(const std::shared_ptr<const transform::WorldModel>& wm,
                                                                 const TaskData& td,
                                                                 const ComponentPosition& reference_position,
                                                                 time::TimePoint time,
                                                                 const ComponentPosition& observe_position) const {
    VectorWithVelocityStamped trans_and_vel;
    trans_and_vel.stamp = time;

    // translation
    std::optional<Eigen::Affine2d> translation_to_point_in_obs_frame =
        this->position.positionObject(wm, td).getCurrentPosition(wm, observe_position.positionObject(wm, td), time);
    std::optional<Eigen::Affine2d> translation_to_ref_in_obs_frame =
        reference_position.positionObject(wm, td).getCurrentPosition(wm, observe_position.positionObject(wm, td), time);

    if (translation_to_point_in_obs_frame.has_value() && translation_to_ref_in_obs_frame.has_value()) {
        trans_and_vel.vec =
            translation_to_point_in_obs_frame->translation() - translation_to_ref_in_obs_frame->translation();
    }

    // velocity
    std::optional<Eigen::Vector3d> velocity =
        this->position.positionObject(wm, td).getVelocity(wm, reference_position.positionObject(wm, td).getFrame(),
                                                          observe_position.positionObject(wm, td).getFrame(), time);
    if (velocity) {
        trans_and_vel.velocity = velocity->head(2);
    }
    return trans_and_vel;
}

std::optional<Eigen::Vector2d> PointShape::getCenter(const std::shared_ptr<const transform::WorldModel>& wm,
                                                     const TaskData& td, time::TimePoint time,
                                                     const ComponentPosition& observe_position) const {
    std::optional<Eigen::Affine2d> vec =
        this->position.positionObject(wm, td).getCurrentPosition(wm, observe_position.positionObject(wm, td), time);
    if (vec.has_value()) return vec->translation();
    return std::nullopt;
}

std::vector<Marker> PointShape::getVisualizationMarker(const std::shared_ptr<const transform::WorldModel>& wm,
                                                       const TaskData& td, time::TimePoint /*time*/) const {
    marker::Circle m(this->position.positionObject(wm, td));
    constexpr double POINT_VIS_RADIUS = 0.05;
    m.setRadius(POINT_VIS_RADIUS);
    m.setFilled(true);
    return {m};
}

}  // namespace luhsoccer::local_planner
