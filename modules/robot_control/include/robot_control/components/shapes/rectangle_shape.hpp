#pragma once

#include "robot_control/components/abstract_shape.hpp"
#include "robot_control/components/shapes/line_shape.hpp"

namespace luhsoccer::robot_control {

class RectangleShape : public AbstractShape {
   public:
    /**
     * @brief Construct a new Rectangle Shape object given the center and height and width
     *
     * @param cs
     * @param center
     * @param width
     * @param height
     * @param filled If the reference is in a filled rect, the vector will be (0,0)
     */
    RectangleShape(const ComponentPosition& center, const DoubleComponentParam& width,
                   const DoubleComponentParam& height, BoolComponentParam filled);

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
    ComponentPosition p1;
    ComponentPosition p2;
    ComponentPosition p3;
    ComponentPosition p4;
    std::array<LineShape, 4> lines;
    BoolComponentParam filled;
    ComponentPosition center;
    DoubleComponentParam height;
    DoubleComponentParam width;
};

}  // namespace luhsoccer::robot_control