#pragma once

#include <utility>

#include "local_planner/skills/abstract_feature.hpp"
#include "config/config_store.hpp"

namespace luhsoccer::local_planner {
class BallTargetFeature : public AbstractTargetFeature {
   public:
    template <typename T>
    BallTargetFeature(const T& shape, const DoubleComponentParam& weight = 1.0,
                      BoolComponentParam ignore_velocity = false,
                      const std::optional<DoubleComponentParam>& k_g = std::nullopt,
                      const std::optional<DoubleComponentParam>& k_v = std::nullopt,
                      const std::optional<DoubleComponentParam>& max_vel_x = std::nullopt,
                      const std::optional<DoubleComponentParam>& max_vel_y = std::nullopt)
        : AbstractTargetFeature(std::make_shared<T>(shape), true, weight, max_vel_x, max_vel_y),
          ignore_velocity(std::move(ignore_velocity)),
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
    BoolComponentParam ignore_velocity;
    DoubleComponentParam k_g;
    DoubleComponentParam k_v;
};

}  // namespace luhsoccer::local_planner