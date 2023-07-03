

#include "local_planner_components/shapes/rectangle_shape.hpp"

namespace luhsoccer::local_planner {

VectorWithVelocityStamped RectangleShape::getTransformToClosestPoint(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
    const ComponentPosition& reference_position, time::TimePoint time,
    const ComponentPosition& observe_position) const {
    VectorWithVelocityStamped trans_and_vel;
    trans_and_vel.stamp = time;

    // translation
    std::vector<LineShape> lines = {this->l1, this->l2, this->l3, this->l4};
    std::optional<Eigen::Vector2d> min_vec;
    for (LineShape& l : lines) {
        std::optional<Eigen::Vector2d> vec =
            l.getTransformToClosestPoint(wm, td, reference_position, time, observe_position).vec;
        if (vec.has_value() && (!min_vec.has_value() || vec->norm() < min_vec->norm())) min_vec = vec.value();
    }

    std::optional<Eigen::Affine2d> trans_center =
        this->center.positionObject(wm, td).getCurrentPosition(wm, center.positionObject(wm, td), time);
    std::optional<Eigen::Affine2d> trans_ref =
        reference_position.positionObject(wm, td).getCurrentPosition(wm, center.positionObject(wm, td), time);
    if (trans_ref.has_value() && trans_center.has_value()) {
        if (filled.val(wm, td) &&
            trans_ref.value().translation().x() <= trans_center.value().translation().x() + width.val(wm, td) / 2 &&
            trans_ref.value().translation().x() >= trans_center.value().translation().x() - width.val(wm, td) / 2 &&
            trans_ref.value().translation().y() <= trans_center.value().translation().y() + height.val(wm, td) / 2 &&
            trans_ref.value().translation().y() >= trans_center.value().translation().y() - height.val(wm, td) / 2) {
            trans_and_vel.vec = {0.0, 0.0};
        } else {
            trans_and_vel.vec = min_vec;
        }
    }

    // velocity
    std::optional<Eigen::Vector3d> velocity =
        this->p1.positionObject(wm, td).getVelocity(wm, reference_position.positionObject(wm, td).getFrame(),
                                                    observe_position.positionObject(wm, td).getFrame(), time);
    if (velocity) {
        trans_and_vel.velocity = velocity->head(2);
    }
    return trans_and_vel;
}

std::optional<Eigen::Vector2d> RectangleShape::getCenter(const std::shared_ptr<const transform::WorldModel>& wm,
                                                         const TaskData& td, time::TimePoint time,
                                                         const ComponentPosition& observe_position) const {
    std::optional<Eigen::Affine2d> vec =
        this->center.positionObject(wm, td).getCurrentPosition(wm, observe_position.positionObject(wm, td), time);
    if (vec.has_value()) return vec->translation();
    return std::nullopt;
}

std::vector<Marker> RectangleShape::getVisualizationMarker(const std::shared_ptr<const transform::WorldModel>& wm,
                                                           const TaskData& td, time::TimePoint /*time*/) const {
    marker::Rect m(this->center.positionObject(wm, td));
    m.setSize({this->height.val(wm, td), this->width.val(wm, td)});
    m.setFilled(this->filled.val(wm, td));
    return {m};
}

}  // namespace luhsoccer::local_planner
