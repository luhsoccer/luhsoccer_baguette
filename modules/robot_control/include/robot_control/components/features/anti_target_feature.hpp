#pragma once

#include <utility>

#include "config/robot_control_config.hpp"
#include "robot_control/components/abstract_feature.hpp"
#include "robot_control/components/component_util.hpp"

namespace luhsoccer::robot_control {
class AntiTargetFeature : public AbstractTargetFeature {
   public:
    template <typename T>
    AntiTargetFeature(const T& shape, DoubleComponentParam influence_distance, const DoubleComponentParam& weight = 1.0,
                      const std::optional<DoubleComponentParam>& k_ip = std::nullopt)
        : AbstractTargetFeature(std::make_shared<T>(shape), false, weight),
          influence_distance(std::move(influence_distance)),
          k_ip(k_ip.has_value() ? k_ip.value() : robotControlConfig().anti_target_k_ip) {}

    [[nodiscard]] std::pair<Eigen::Vector2d, Eigen::Vector2d> calcArtificialDesiredVelocity(
        const ComponentData& comp_data, const std::vector<std::shared_ptr<const AbstractShape>>& restricted_areas,
        const transform::Position& robot_position, const transform::Position& observe_position = "") const override;

    [[nodiscard]] marker::Color getVisualizationColor(const ComponentData& /*comp_data*/) const override {
        return RC_RED;
    };

   private:
    DoubleComponentParam influence_distance;
    DoubleComponentParam k_ip;
};
}  // namespace luhsoccer::robot_control