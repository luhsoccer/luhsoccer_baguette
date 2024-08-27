#pragma once

#include <utility>

#include "robot_control/components/abstract_shape.hpp"
#include "robot_control/components/component_util.hpp"

namespace luhsoccer::robot_control {

class LineShape : public AbstractShape {
   public:
    /**
     * @brief Construct a new Line Shape object
     *
     * @param start
     * @param end
     * @param cutoff_start makes the line longer/shorter by the given length (negativ for longer, positiv for shorter)
     * @param cutoff_end makes the line longer/shorter by the given length (negativ for longer, positiv for shorter)
     */
    LineShape(ComponentPosition start, ComponentPosition end, DoubleComponentParam cutoff_start = {0.0},
              DoubleComponentParam cutoff_end = {0.0})
        : start(std::move(start)),
          end(std::move(end)),
          cutoff_start(std::move(cutoff_start)),
          cutoff_end(std::move(cutoff_end)){};

    /**
     * @brief Construct a new Line Shape object given a point and an angle
     *
     * @param point A point on the line
     * @param angle The angle of the line
     * @param length The length of the line, default is 1
     * @param cutoff_start Cutoff for point
     * @param cutoff_end Cuttoff for the end point, which has a distance of length to point
     */
    LineShape(const ComponentPosition& point, const DoubleComponentParam& angle, const DoubleComponentParam& length,
              DoubleComponentParam cutoff_start = {0.0}, DoubleComponentParam cutoff_end = {0.0});

    [[nodiscard]] VectorWithVelocityStamped getTransformToClosestPoint(
        const ComponentData& comp_data, const transform::Position& robot_position,
        const transform::Position& observe_position = "") const override;

    [[nodiscard]] std::optional<Eigen::Vector2d> getCenter(
        const ComponentData& comp_data, const transform::Position& observe_position = "") const override;

   private:
    ComponentPosition start;
    ComponentPosition end;
    DoubleComponentParam cutoff_start;
    DoubleComponentParam cutoff_end;
};

}  // namespace luhsoccer::robot_control