#pragma once

#include <utility>

#include "local_planner/skills/abstract_feature.hpp"
#include "config/config_store.hpp"
#include "local_planner_components/shapes/point_shape.hpp"

namespace luhsoccer::local_planner {
constexpr double DEFAULT_ROTATIONAL_TOLERANCE = 3.0 * L_PI / 180;
class TurnAroundBallFeature : public AbstractTargetFeature {
   public:
    TurnAroundBallFeature(ComponentPosition align_position,
                          DoubleComponentParam rotational_tolerance = DEFAULT_ROTATIONAL_TOLERANCE)
        : AbstractTargetFeature(std::make_shared<const PointShape>(ComponentPosition("ball")), true, 1.0),
          rotational_tolerance(std::move(rotational_tolerance)),
          align_position(std::move(align_position)){};

    [[nodiscard]] Eigen::Vector2d calcArtificialDesiredVelocity(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const time::TimePoint& time = time::TimePoint(0),
        const ComponentPosition& observe_position = "") const override;

    [[nodiscard]] bool isReached(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                                 const RobotIdentifier& robot,
                                 const time::TimePoint& time = time::TimePoint(0)) const override;

    [[nodiscard]] marker::Color getVisualizationColor(const std::shared_ptr<const transform::WorldModel>& /*wm*/,
                                                      const TaskData& /*td*/, const RobotIdentifier& /*robot*/,
                                                      time::TimePoint /*time*/ = time::TimePoint(0)) const override {
        return marker::Color::GREEN();
    };

   private:
    DoubleComponentParam rotational_tolerance;
    ComponentPosition align_position;
};

}  // namespace luhsoccer::local_planner