#include "robot_control/components/shapes/compose_shape.hpp"
#include <numeric>

#include "robot_control/components/component_data.hpp"

namespace luhsoccer::robot_control {

VectorWithVelocityStamped ComposeShape::getTransformToClosestPoint(const ComponentData& comp_data,
                                                                   const transform::Position& robot_position,
                                                                   const transform::Position& observe_position) const {
    std::vector<VectorWithVelocityStamped> vecs;
    vecs.reserve(this->shapes.size());
    for (const std::shared_ptr<const AbstractShape>& s : this->shapes) {
        VectorWithVelocityStamped v = s->getTransformToClosestPoint(comp_data, robot_position, observe_position);
        if (v.vec.has_value()) vecs.push_back(v);
    }

    VectorWithVelocityStamped closest;
    closest.stamp = comp_data.time;
    for (VectorWithVelocityStamped& v : vecs) {
        if (v.vec.has_value()) {
            if (!closest.vec.has_value() || (closest.vec.has_value() && closest.vec->norm() > v.vec->norm()))
                closest = v;
        }
    }

    return closest;
}

std::optional<Eigen::Vector2d> ComposeShape::getCenter(const ComponentData& comp_data,
                                                       const transform::Position& observe_position) const {
    Eigen::Vector2d vec_sum = Eigen::Vector2d::Zero();
    int vecs = 0;
    for (const std::shared_ptr<const AbstractShape>& s : this->shapes) {
        std::optional<Eigen::Vector2d> vec = s->getCenter(comp_data, observe_position);
        if (vec.has_value()) {
            vecs++;
            vec_sum += vec.value();
        }
    }

    if (vecs > 0) return vec_sum / vecs;
    return std::nullopt;
}

bool ComposeShape::hasArea(const ComponentData& comp_data) const {
    return std::any_of(
        this->shapes.begin(), this->shapes.end(),
        [&comp_data](const std::shared_ptr<const AbstractShape>& shape) { return shape->hasArea(comp_data); });
}

double ComposeShape::getAreaSize(const ComponentData& comp_data) const {
    return std::accumulate(this->shapes.begin(), this->shapes.end(), 0.0,
                           [&comp_data](double sum, const std::shared_ptr<const AbstractShape>& shape) {
                               return sum + shape->getAreaSize(comp_data);
                           });
}

std::vector<Eigen::Vector2d> ComposeShape::getPointsOnBrim(const ComponentData& comp_data, double spacing,
                                                           const transform::Position& observe_position,
                                                           double margin) const {
    std::vector<Eigen::Vector2d> brim_points;
    for (const auto& shape : this->shapes) {
        auto ps = shape->getPointsOnBrim(comp_data, spacing, observe_position, margin);
        brim_points.insert(brim_points.end(), ps.begin(), ps.end());
    }
    return brim_points;
}

bool ComposeShape::isPointInArea(const Eigen::Vector2d& point, const ComponentData& comp_data,
                                 const transform::Position& observe_position) const {
    return std::any_of(this->shapes.begin(), this->shapes.end(),
                       [&point, &comp_data, &observe_position](const std::shared_ptr<const AbstractShape>& shape) {
                           return shape->isPointInArea(point, comp_data, observe_position);
                       });
}

}  // namespace luhsoccer::robot_control
