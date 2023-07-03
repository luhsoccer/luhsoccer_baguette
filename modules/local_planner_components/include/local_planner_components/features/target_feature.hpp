#pragma once

#include <utility>

#include "local_planner/skills/abstract_feature.hpp"
#include "config/config_store.hpp"

namespace luhsoccer::local_planner {
constexpr double DEFAULT_TRANSLATIONAL_TOLERANCE = 0.05;
class TargetFeature : public AbstractTargetFeature {
   public:
    template <typename T>
    TargetFeature(const T& shape, DoubleComponentParam translational_tolerance = DEFAULT_TRANSLATIONAL_TOLERANCE,
                  const DoubleComponentParam& weight = 1.0, BoolComponentParam ignore_velocity = false,
                  BoolComponentParam velocity_zero_for_reach = false,
                  const std::optional<DoubleComponentParam>& k_g = std::nullopt,
                  const std::optional<DoubleComponentParam>& k_v = std::nullopt,
                  const std::optional<DoubleComponentParam>& max_vel_x = std::nullopt,
                  const std::optional<DoubleComponentParam>& max_vel_y = std::nullopt)
        : AbstractTargetFeature(std::make_shared<T>(shape), true, weight, max_vel_x, max_vel_y),
          translational_tolerance(std::move(translational_tolerance)),
          ignore_velocity(std::move(ignore_velocity)),
          velocity_zero_for_reach(std::move(velocity_zero_for_reach)),
          k_g(localPlannerConfig().feature_target_k_g),
          k_v(localPlannerConfig().feature_target_k_v) {
        if (k_g) {
            this->k_g = *k_g;
        }
        if (k_v) {
            this->k_v = *k_v;
        }
    }

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
    static constexpr double ZERO_VELOCITY_TOLERANCE = 0.1;

    DoubleComponentParam translational_tolerance;
    BoolComponentParam ignore_velocity;
    BoolComponentParam velocity_zero_for_reach;
    DoubleComponentParam k_g;
    DoubleComponentParam k_v;
};

}  // namespace luhsoccer::local_planner