#include "avoid_other_robots.hpp"
#include "local_planner_components/features/robot_cf_obstacle.hpp"

namespace luhsoccer::local_planner {

std::vector<std::shared_ptr<const AbstractCFObstacle>> AvoidOtherRobotsConstraint ::getAdditionalObstacleFeaturesImpl()
    const {
    std::vector<std::shared_ptr<const AbstractCFObstacle>> features;
    for (const auto other_robot : transform::RobotDataStorage::generateAllPossibleRobots(MAX_ROBOTS_PER_TEAM)) {
        DoubleComponentParam w(
            CALLBACK,
            [other_robot](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
                auto other_robot_data = wm->getRobotData(other_robot);

                if (other_robot != td.robot && other_robot_data.has_value() && other_robot_data->on_field) {
                    return 1.0;
                } else {
                    return 0.0;
                }
            });
        features.emplace_back(std::make_shared<RobotCFObstacle>(other_robot, w));
    }
    return features;
}

}  // namespace luhsoccer::local_planner