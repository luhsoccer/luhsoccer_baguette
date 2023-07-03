

#include "local_planner_components/shapes/circle_shape.hpp"

namespace luhsoccer::local_planner {

VectorWithVelocityStamped CircleShape::getTransformToClosestPoint(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
    const ComponentPosition& reference_position, time::TimePoint time,
    const ComponentPosition& observe_position) const {
    VectorWithVelocityStamped trans_and_vel;
    trans_and_vel.stamp = time;

    // translation
    std::optional<Eigen::Affine2d> translation_to_point_in_obs_frame =
        this->position.positionObject(wm, td).getCurrentPosition(wm, observe_position.positionObject(wm, td), time);
    std::optional<Eigen::Affine2d> translation_to_ref_in_obs_frame =
        reference_position.positionObject(wm, td).getCurrentPosition(wm, observe_position.positionObject(wm, td), time);

    if (translation_to_point_in_obs_frame.has_value() && translation_to_ref_in_obs_frame.has_value()) {
        Eigen::Vector2d translation_between_ref_and_point =
            translation_to_point_in_obs_frame->translation() - translation_to_ref_in_obs_frame->translation();

        if (translation_between_ref_and_point.norm() == 0.0) {
            trans_and_vel.vec = {0.0, 0.0};
        } else {
            double factor = 1 - this->radius.val(wm, td) / translation_between_ref_and_point.norm();

            trans_and_vel.vec = translation_between_ref_and_point * factor;

            if (this->filled.val(wm, td) && factor < 0.0) {
                trans_and_vel.vec = {0.0, 0.0};
            }
        }
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

std::optional<Eigen::Vector2d> CircleShape::getCenter(const std::shared_ptr<const transform::WorldModel>& wm,
                                                      const TaskData& td, time::TimePoint time,
                                                      const ComponentPosition& observe_position) const {
    std::optional<Eigen::Affine2d> vec =
        this->position.positionObject(wm, td).getCurrentPosition(wm, observe_position.positionObject(wm, td), time);
    if (vec.has_value()) return vec->translation();
    return std::nullopt;
}

std::vector<Marker> CircleShape::getVisualizationMarker(const std::shared_ptr<const transform::WorldModel>& wm,
                                                        const TaskData& td, time::TimePoint /*time*/) const {
    marker::Circle m(this->position.positionObject(wm, td));
    m.setRadius(this->radius.val(wm, td));
    m.setFilled(this->filled.val(wm, td));
    return {m};
}

}  // namespace luhsoccer::local_planner
