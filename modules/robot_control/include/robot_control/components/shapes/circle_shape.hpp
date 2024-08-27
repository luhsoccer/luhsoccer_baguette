#pragma once

#include "robot_control/components/abstract_shape.hpp"
#include "robot_control/components/component_util.hpp"

namespace luhsoccer::robot_control {

class CircleShape : public AbstractShape {
   public:
    CircleShape(ComponentPosition position, DoubleComponentParam radius, BoolComponentParam filled)
        : position(std::move(position)), radius(std::move(radius)), filled(std::move(filled)){};

    [[nodiscard]] VectorWithVelocityStamped getTransformToClosestPoint(
        const ComponentData& comp_data, const transform::Position& robot_position,
        const transform::Position& observe_position = "") const override;

    [[nodiscard]] std::optional<Eigen::Vector2d> getCenter(
        const ComponentData& comp_data, const transform::Position& observe_position = "") const override;

    [[nodiscard]] bool hasArea(const ComponentData& comp_data) const override { return filled.val(comp_data); }

    [[nodiscard]] double getAreaSize(const ComponentData& comp_data) const override;

    [[nodiscard]] std::vector<Eigen::Vector2d> getPointsOnBrim(const ComponentData& comp_data,
                                                               double spacing = DEFAULT_BRIM_SPACING,
                                                               const transform::Position& observe_position = "",
                                                               double margin = 0.0) const override;

    [[nodiscard]] bool isPointInArea(const Eigen::Vector2d& point, const ComponentData& comp_data,
                                     const transform::Position& observe_position = "") const override;

   private:
    ComponentPosition position;
    DoubleComponentParam radius;
    BoolComponentParam filled;
};

}  // namespace luhsoccer::robot_control