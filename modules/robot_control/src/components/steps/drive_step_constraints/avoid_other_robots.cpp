#include "avoid_other_robots.hpp"
#include "robot_control/components/component_data.hpp"
#include "robot_control/components/features/obstacle_feature.hpp"

namespace luhsoccer::robot_control {

std::vector<std::shared_ptr<const AbstractObstacle>> AvoidOtherRobotsConstraint ::getAdditionalObstacleFeaturesImpl()
    const {
    std::vector<std::shared_ptr<const AbstractObstacle>> features;
    for (const auto other_robot : transform::RobotDataStorage::generateAllPossibleRobots(MAX_ROBOTS_PER_TEAM)) {
        DoubleComponentParam w(CALLBACK, [other_robot](const ComponentData& comp_data, const ComponentUid&) -> double {
            /// @todo removed defense area disable
            auto other_robot_data = comp_data.wm->getRobotData(other_robot);

            if (other_robot != comp_data.td.robot && other_robot_data.has_value() && other_robot_data->on_field) {
                return 1.0;
            } else {
                return 0.0;
            }
        });
        features.emplace_back(std::make_shared<RobotObstacleFeature>(other_robot, w));
    }
    return features;
}

}  // namespace luhsoccer::robot_control