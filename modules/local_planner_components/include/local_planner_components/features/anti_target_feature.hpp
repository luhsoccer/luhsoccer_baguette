#pragma once

#include <utility>

#include "local_planner/skills/abstract_feature.hpp"
#include "config/config_store.hpp"

namespace luhsoccer::local_planner {
class AntiTargetFeature : public AbstractTargetFeature {
   public:
    template <typename T>
    AntiTargetFeature(const T& shape, DoubleComponentParam influence_distance, const DoubleComponentParam& weight = 1.0,
                      const std::optional<DoubleComponentParam>& max_vel_x = std::nullopt,
                      const std::optional<DoubleComponentParam>& max_vel_y = std::nullopt,
                      const std::optional<DoubleComponentParam>& feature_k = std::nullopt)
        : AbstractTargetFeature(std::make_shared<T>(shape), false, weight, max_vel_x, max_vel_y),
          influence_distance(std::move(influence_distance)),
          feature_k(feature_k.has_value() ? feature_k.value() : localPlannerConfig().feature_anti_target_k) {}

    [[nodiscard]] Eigen::Vector2d calcArtificialDesiredVelocity(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const time::TimePoint& time = time::TimePoint(0),
        const ComponentPosition& observe_position = "") const override;

    [[nodiscard]] marker::Color getVisualizationColor(const std::shared_ptr<const transform::WorldModel>& /*wm*/,
                                                      const TaskData& /*td*/, const RobotIdentifier& /*robot*/,
                                                      time::TimePoint /*time*/ = time::TimePoint(0)) const override {
        return marker::Color::RED();
    };

   private:
    DoubleComponentParam influence_distance;
    DoubleComponentParam feature_k;
};
}  // namespace luhsoccer::local_planner