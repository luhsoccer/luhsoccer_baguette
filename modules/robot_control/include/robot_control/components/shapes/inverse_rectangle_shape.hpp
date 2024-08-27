#pragma once

#include "robot_control/components/shapes/rectangle_shape.hpp"

namespace luhsoccer::robot_control {

class InverseRectangleShape : public RectangleShape {
   public:
    InverseRectangleShape(const ComponentPosition& center, const DoubleComponentParam& width,
                          const DoubleComponentParam& height)
        : RectangleShape(center, width, height, false){};

    [[nodiscard]] bool hasArea(const ComponentData&) const override { return true; }

    [[nodiscard]] bool isPointInArea(const Eigen::Vector2d& point, const ComponentData& comp_data,
                                     const transform::Position& observe_position = "") const override {
        return !RectangleShape::isPointInArea(point, comp_data, observe_position);
    }

    [[nodiscard]] std::vector<Eigen::Vector2d> getPointsOnBrim(const ComponentData& comp_data,
                                                               double spacing = DEFAULT_BRIM_SPACING,
                                                               const transform::Position& observe_position = "",
                                                               double margin = 0.0) const override {
        return RectangleShape::getPointsOnBrim(comp_data, spacing, observe_position, -margin);
    };

    [[nodiscard]] bool isGroupable(const ComponentData& /*comp_data*/) const override { return false; }
};
}  // namespace luhsoccer::robot_control