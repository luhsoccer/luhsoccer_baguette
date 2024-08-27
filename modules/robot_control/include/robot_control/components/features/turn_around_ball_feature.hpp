#pragma once

#include <utility>

#include "robot_control/components/abstract_feature.hpp"
#include "robot_control/components/shapes/point_shape.hpp"

namespace luhsoccer::robot_control {
constexpr double DEFAULT_ROTATIONAL_TOLERANCE = 3.0 * L_PI / 180;
class TurnAroundBallFeature : public AbstractTargetFeature {
   public:
    explicit TurnAroundBallFeature(ComponentPosition align_position,
                                   DoubleComponentParam rotational_tolerance = DEFAULT_ROTATIONAL_TOLERANCE)
        : AbstractTargetFeature(std::make_shared<const PointShape>(ComponentPosition("ball")), true, 1.0),
          rotational_tolerance(std::move(rotational_tolerance)),
          align_position(std::move(align_position)){};

    [[nodiscard]] std::pair<Eigen::Vector2d, Eigen::Vector2d> calcArtificialDesiredVelocity(
        const ComponentData& comp_data, const std::vector<std::shared_ptr<const AbstractShape>>& restricted_areas,
        const transform::Position& robot_position, const transform::Position& observe_position = "") const override;

    [[nodiscard]] bool isReached(const ComponentData& comp_data) const override;

    [[nodiscard]] marker::Color getVisualizationColor(const ComponentData& /*comp_data*/) const override {
        return RC_GREEN;
    };

   private:
    DoubleComponentParam rotational_tolerance;
    ComponentPosition align_position;
};

}  // namespace luhsoccer::robot_control