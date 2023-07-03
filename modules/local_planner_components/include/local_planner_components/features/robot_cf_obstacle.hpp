
#pragma once

#include "local_planner/skills/abstract_feature.hpp"
#include "local_planner_components/shapes/point_shape.hpp"

namespace luhsoccer::local_planner {

class RobotCFObstacle : public AbstractCFObstacle {
   public:
    template <typename T>
    RobotCFObstacle(const T& shape, const DoubleComponentParam& weight = 1.0,
                    const std::optional<DoubleComponentParam>& influence_distance = std::nullopt,
                    const std::optional<DoubleComponentParam>& ataka_influence_distance = std::nullopt,
                    const std::optional<DoubleComponentParam>& critical_distance = std::nullopt,
                    const std::optional<DoubleComponentParam>& k_cf = std::nullopt,
                    const std::optional<DoubleComponentParam>& k_ataka = std::nullopt,
                    const std::optional<DoubleComponentParam>& max_vel_x = std::nullopt,
                    const std::optional<DoubleComponentParam>& max_vel_y = std::nullopt)
        : AbstractCFObstacle(std::make_shared<T>(shape), weight, max_vel_x, max_vel_y, critical_distance,
                             influence_distance),
          ataka_influence_distance(ataka_influence_distance.has_value()
                                       ? ataka_influence_distance.value()
                                       : localPlannerConfig().feature_robot_obstacle_ataka_influence_distance),
          k_cf(k_cf.has_value() ? k_cf.value() : localPlannerConfig().feature_robot_obstacle_k_cf),
          k_ataka(k_ataka.has_value() ? k_ataka.value() : localPlannerConfig().feature_robot_obstacle_k_ataka) {}

    RobotCFObstacle(RobotIdentifier robot, const DoubleComponentParam& weight = 1.0,
                    const std::optional<DoubleComponentParam>& robot_radius = std::nullopt,
                    const std::optional<DoubleComponentParam>& influence_distance = std::nullopt,
                    const std::optional<DoubleComponentParam>& ataka_influence_distance = std::nullopt,
                    const std::optional<DoubleComponentParam>& critical_distance = std::nullopt,
                    const std::optional<DoubleComponentParam>& k_cf = std::nullopt,
                    const std::optional<DoubleComponentParam>& k_ataka = std::nullopt,
                    const std::optional<DoubleComponentParam>& max_vel_x = std::nullopt,
                    const std::optional<DoubleComponentParam>& max_vel_y = std::nullopt);

    [[nodiscard]] std::optional<RobotIdentifier> getRobot() const override { return this->shape_robot; }

    [[nodiscard]] marker::Color getVisualizationColor(const std::shared_ptr<const transform::WorldModel>& /*wm*/,
                                                      const TaskData& /*td*/, const RobotIdentifier& /*robot*/,
                                                      time::TimePoint /*time*/ = time::TimePoint(0)) const override {
        return marker::Color::ORANGE();
    };
    [[nodiscard]] std::vector<Marker> getVisualizationMarker(const std::shared_ptr<const transform::WorldModel>& wm,
                                                             const TaskData& td, const RobotIdentifier& robot,
                                                             time::TimePoint time = time::TimePoint(0)) const override;

   private:
    const DoubleComponentParam ataka_influence_distance;
    const DoubleComponentParam k_cf;
    const DoubleComponentParam k_ataka;
    std::optional<RobotIdentifier> shape_robot;
};
}  // namespace luhsoccer::local_planner