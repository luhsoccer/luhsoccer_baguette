#pragma once

#include "robot_control/components/abstract_feature.hpp"

namespace luhsoccer::robot_control {

class ObstacleFeature : public AbstractObstacle {
   public:
    template <typename T>
    explicit ObstacleFeature(const T& shape, const DoubleComponentParam& weight = 1.0,
                             const std::optional<DoubleComponentParam>& critical_distance = std::nullopt,
                             const std::optional<DoubleComponentParam>& influence_distance = std::nullopt,
                             DoubleComponentParam k_obstacle = 1.0)
        : AbstractObstacle(std::make_shared<T>(shape), weight, critical_distance, influence_distance),
          k_obstacle(std::move(k_obstacle)) {}

    [[nodiscard]] marker::Color getVisualizationColor(const ComponentData&) const override { return RC_ORANGE; };

    [[nodiscard]] std::optional<Eigen::Vector2d> getVelocityCommand(
        const ComponentData& comp_data, const Eigen::Vector2d& command_velocity, bool bypass_direction,
        double mean_target_distance, const transform::Position& robot_position,
        const transform::Position& observe_position = "") const override;

   protected:
    DoubleComponentParam k_obstacle;

    // configuration methods
   public:
    void setKObstacle(DoubleComponentParam k_obstacle) { this->k_obstacle = std::move(k_obstacle); }
};

class RobotObstacleFeature : public ObstacleFeature {
   public:
    explicit RobotObstacleFeature(const RobotIdentifier& robot, const DoubleComponentParam& weight = 1.0,
                                  const std::optional<DoubleComponentParam>& critical_distance = std::nullopt,
                                  const std::optional<DoubleComponentParam>& influence_distance = std::nullopt,
                                  DoubleComponentParam k_obstacle = 1.0);

    [[nodiscard]] std::optional<RobotIdentifier> getRobot() const override { return this->robot; }

   private:
    RobotIdentifier robot;
};
}  // namespace luhsoccer::robot_control