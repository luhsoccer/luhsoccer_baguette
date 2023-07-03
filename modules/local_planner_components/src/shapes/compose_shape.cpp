

#include "local_planner_components/shapes/compose_shape.hpp"

namespace luhsoccer::local_planner {

VectorWithVelocityStamped ComposeShape::getTransformToClosestPoint(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
    const ComponentPosition& reference_position, time::TimePoint time,
    const ComponentPosition& observe_position) const {
    VectorWithVelocityStamped trans_and_vel;
    trans_and_vel.stamp = time;

    std::vector<VectorWithVelocityStamped> vecs;
    vecs.reserve(this->shapes.size());
    for (const std::shared_ptr<const AbstractShape>& s : this->shapes) {
        VectorWithVelocityStamped v = s->getTransformToClosestPoint(wm, td, reference_position, time, observe_position);
        if (v.vec.has_value()) vecs.push_back(v);
    }

    VectorWithVelocityStamped next;
    next.stamp = time;
    for (VectorWithVelocityStamped& v : vecs) {
        if (v.vec.has_value()) {
            if (!next.vec.has_value() || (next.vec.has_value() && next.vec->norm() > v.vec->norm())) next = v;
        }
    }

    return next;
}

std::optional<Eigen::Vector2d> ComposeShape::getCenter(const std::shared_ptr<const transform::WorldModel>& wm,
                                                       const TaskData& td, time::TimePoint time,
                                                       const ComponentPosition& observe_position) const {
    Eigen::Vector2d vec_sum = Eigen::Vector2d::Zero();
    int vecs = 0;
    for (const std::shared_ptr<const AbstractShape>& s : this->shapes) {
        std::optional<Eigen::Vector2d> vec = s->getCenter(wm, td, time, observe_position);
        if (vec.has_value()) {
            vecs++;
            vec_sum += vec.value();
        }
    }

    if (vecs > 0) return vec_sum / vecs;
    return std::nullopt;
}

std::vector<Marker> ComposeShape::getVisualizationMarker(const std::shared_ptr<const transform::WorldModel>& wm,
                                                         const TaskData& td, time::TimePoint time) const {
    std::vector<Marker> ms;
    for (const auto& shape : this->shapes) {
        std::vector<Marker> m = shape->getVisualizationMarker(wm, td, time);
        ms.insert(ms.end(), m.begin(), m.end());
    }
    return ms;
}

}  // namespace luhsoccer::local_planner
