#include "robot_control/components/shapes/point_shape.hpp"
#include "robot_control/components/component_data.hpp"
#include "marker_adapter.hpp"
#include "marker_service/marker.hpp"

namespace luhsoccer::robot_control {
VectorWithVelocityStamped PointShape::getTransformToClosestPoint(const ComponentData& comp_data,
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
        trans_and_vel.vec =
            translation_to_point_in_obs_frame->translation() - translation_to_ref_in_obs_frame->translation();
    }

    // velocity
    std::optional<Eigen::Vector3d> velocity = this->position.positionObject(comp_data).getVelocity(
        comp_data.wm, robot_position, observe_position, comp_data.time);
    if (velocity) {
        trans_and_vel.velocity = velocity->head(2);
    }

    // visualization
    marker::Circle m(this->position.positionObject(comp_data), "", 0);
    constexpr double POINT_VIS_DIAMETER = 0.05;
    m.setRadius(POINT_VIS_DIAMETER / 2);
    m.setFilled(true);
    comp_data.ma.displayMarker(m);

    return trans_and_vel;
}

std::optional<Eigen::Vector2d> PointShape::getCenter(const ComponentData& comp_data,
                                                     const transform::Position& observe_position) const {
    auto pos =
        this->position.positionObject(comp_data).getCurrentPosition(comp_data.wm, observe_position, comp_data.time);
    if (!pos.has_value()) return std::nullopt;

    return pos->translation();
}
}  // namespace luhsoccer::robot_control