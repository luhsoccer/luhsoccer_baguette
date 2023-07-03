#include <utility>

#include "local_planner_components/features/robot_cf_obstacle.hpp"
#include "time/calc_time_stopwatch.hpp"
#include "logger/logger.hpp"
#include "visit.hpp"
namespace luhsoccer::local_planner {

RobotCFObstacle::RobotCFObstacle(RobotIdentifier robot, const DoubleComponentParam& weight,
                                 const std::optional<DoubleComponentParam>& /*robot_radius*/,
                                 const std::optional<DoubleComponentParam>& influence_distance,
                                 const std::optional<DoubleComponentParam>& ataka_influence_distance,
                                 const std::optional<DoubleComponentParam>& critical_distance,
                                 const std::optional<DoubleComponentParam>& k_cf,
                                 const std::optional<DoubleComponentParam>& k_ataka,
                                 const std::optional<DoubleComponentParam>& max_vel_x,
                                 const std::optional<DoubleComponentParam>& max_vel_y)
    /// @todo change point shape to circle
    : AbstractCFObstacle(std::make_shared<PointShape>(robot), weight, max_vel_x, max_vel_y, critical_distance,
                         influence_distance),
      ataka_influence_distance(ataka_influence_distance.has_value()
                                   ? ataka_influence_distance.value()
                                   : localPlannerConfig().feature_robot_obstacle_ataka_influence_distance),
      k_cf(k_cf.has_value() ? k_cf.value() : localPlannerConfig().feature_robot_obstacle_k_cf),
      k_ataka(k_ataka.has_value() ? k_ataka.value() : localPlannerConfig().feature_robot_obstacle_k_ataka),
      shape_robot(robot) {}

std::vector<Marker> RobotCFObstacle::getVisualizationMarker(const std::shared_ptr<const transform::WorldModel>& wm,
                                                            const TaskData& td, const RobotIdentifier& robot,
                                                            time::TimePoint time) const {
    std::vector<Marker> ms = AbstractFeature::getVisualizationMarker(wm, td, robot, time);
    const auto& viz_options = config_provider::ConfigProvider::getConfigStore().local_planner_visualization_config;
    if (viz_options.rotation_vectors_display) {
        for (auto& m : ms) {
            auto visitor = overload{[](marker::Marker& marker) {
                constexpr double OVER_ROBOT_HEIGHT = 0.3;
                marker.setHeight(OVER_ROBOT_HEIGHT);
            }};
            std::visit(visitor, m);
        }
    }
    return ms;
}
}  // namespace luhsoccer::local_planner