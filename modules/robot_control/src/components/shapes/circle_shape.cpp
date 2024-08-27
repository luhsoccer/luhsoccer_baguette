#include "robot_control/components/shapes/circle_shape.hpp"

#include "robot_control/components/component_data.hpp"
#include "marker_adapter.hpp"
#include "marker_service/marker.hpp"

namespace luhsoccer::robot_control {

VectorWithVelocityStamped CircleShape::getTransformToClosestPoint(const ComponentData& comp_data,
                                                                  const transform::Position& robot_position,
                                                                  const transform::Position& observe_position) const {
    VectorWithVelocityStamped trans_and_vel;
    trans_and_vel.stamp = comp_data.time;

    // translation
    std::optional<Eigen::Affine2d> translation_to_point_in_obs_frame =
        this->position.positionObject(comp_data).getCurrentPosition(comp_data.wm, observe_position, comp_data.time);
    std::optional<Eigen::Affine2d> translation_to_ref_in_obs_frame =
        robot_position.getCurrentPosition(comp_data.wm, observe_position, comp_data.time);

    if (translation_to_point_in_obs_frame.has_value() && translation_to_ref_in_obs_frame.has_value()) {
        Eigen::Vector2d translation_between_ref_and_point =
            translation_to_point_in_obs_frame->translation() - translation_to_ref_in_obs_frame->translation();

        if (translation_between_ref_and_point.norm() == 0.0) {
            trans_and_vel.vec = {0.0, 0.0};
        } else {
            double factor = 1 - this->radius.val(comp_data) / translation_between_ref_and_point.norm();

            trans_and_vel.vec = translation_between_ref_and_point * factor;

            if (this->filled.val(comp_data) && factor < 0.0) {
                trans_and_vel.vec = {0.0, 0.0};
            }
        }
    }

    // velocity
    std::optional<Eigen::Vector3d> velocity = this->position.positionObject(comp_data).getVelocity(
        comp_data.wm, robot_position.getFrame(), observe_position.getFrame(), comp_data.time);
    if (velocity) {
        trans_and_vel.velocity = velocity->head(2);
    }

    marker::Circle m(this->position.positionObject(comp_data));
    m.setRadius(this->radius.val(comp_data));
    m.setFilled(this->filled.val(comp_data));

    comp_data.ma.displayMarker(m);

    return trans_and_vel;
}

std::optional<Eigen::Vector2d> CircleShape::getCenter(const ComponentData& comp_data,
                                                      const transform::Position& observe_position) const {
    std::optional<Eigen::Affine2d> vec =
        this->position.positionObject(comp_data).getCurrentPosition(comp_data.wm, observe_position, comp_data.time);
    if (vec.has_value()) return vec->translation();
    return std::nullopt;
}

std::vector<Eigen::Vector2d> CircleShape::getPointsOnBrim(const ComponentData& comp_data, double spacing,
                                                          const transform::Position& observe_position,
                                                          double margin) const {
    std::vector<Eigen::Vector2d> brim_points;
    double radius_val = this->radius.val(comp_data) + margin;
    int n_points = static_cast<int>(2 * L_PI * radius_val / spacing);
    brim_points.reserve(n_points);

    auto center_pose =
        this->position.positionObject(comp_data).getCurrentPosition(comp_data.wm, observe_position, comp_data.time);
    if (!center_pose.has_value()) return {};
    Eigen::Vector2d center = center_pose->translation();
    for (int i = 0; i < n_points; i++) {
        double angle = 2 * L_PI * i / n_points;
        brim_points.emplace_back(center.x() + cos(angle) * radius_val, center.y() + sin(angle) * radius_val);
    }
    return brim_points;
}

bool CircleShape::isPointInArea(const Eigen::Vector2d& point, const ComponentData& comp_data,
                                const transform::Position& observe_position) const {
    if (!this->filled.val(comp_data)) return false;

    auto center =
        this->position.positionObject(comp_data).getCurrentPosition(comp_data.wm, observe_position, comp_data.time);

    if (!center.has_value()) return false;

    return (point - center->translation()).norm() < this->radius.val(comp_data);
}

double CircleShape::getAreaSize(const ComponentData& comp_data) const {
    if (this->filled.val(comp_data)) {
        return L_PI * std::pow(this->radius.val(comp_data), 2);
    }
    return 0.0;
}

}  // namespace luhsoccer::robot_control
