#pragma once

#include "robot_control/components/abstract_shape.hpp"
#include "robot_control/components/component_util.hpp"

namespace luhsoccer::robot_control {

class ArcShape : public AbstractShape {
   public:
    ArcShape(ComponentPosition center, DoubleComponentParam radius, DoubleComponentParam local_angle)
        : center(std::move(center)), radius(std::move(radius)), local_angle(std::move(local_angle)){};

    [[nodiscard]] VectorWithVelocityStamped getTransformToClosestPoint(
        const ComponentData& comp_data, const transform::Position& robot_position,
        const transform::Position& observe_position = "") const override;

    [[nodiscard]] std::optional<Eigen::Vector2d> getCenter(
        const ComponentData& comp_data, const transform::Position& observe_position = "") const override;

   private:
    [[nodiscard]] std::optional<std::pair<double, Eigen::Vector2d>> defineStart(
        const ComponentData& comp_data, const transform::Position& observe_position = "") const;

    [[nodiscard]] std::optional<std::pair<double, Eigen::Vector2d>> defineEnd(
        const ComponentData& comp_data, const transform::Position& observe_position = "") const;

    ComponentPosition center;
    DoubleComponentParam radius;
    DoubleComponentParam local_angle;
};

}  // namespace luhsoccer::robot_control