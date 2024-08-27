#pragma once

#include "robot_control/components/abstract_shape.hpp"
#include "robot_control/components/component_util.hpp"

namespace luhsoccer::robot_control {

class PointShape : public AbstractShape {
   public:
    explicit PointShape(ComponentPosition position) : position(std::move(position)){};

    [[nodiscard]] VectorWithVelocityStamped getTransformToClosestPoint(
        const ComponentData& comp_data, const transform::Position& robot_position,
        const transform::Position& observe_position = "") const override;

    [[nodiscard]] std::optional<Eigen::Vector2d> getCenter(
        const ComponentData& comp_data, const transform::Position& observe_position = "") const override;

   private:
    ComponentPosition position;
};

}  // namespace luhsoccer::robot_control