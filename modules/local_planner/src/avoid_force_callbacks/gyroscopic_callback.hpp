#pragma once

#include "local_planner/avoidance_manager_unit.hpp"

namespace luhsoccer::local_planner {
class AvoidanceManager;

AvoidForceResult getGyroscopicForce(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                                    const RobotIdentifier& robot, AvoidanceManager& am, const time::TimePoint time,
                                    const std::vector<std::shared_ptr<const AbstractCFObstacle>>& obstacles,
                                    const Eigen::Vector2d& target_force, const Eigen::Vector2d& max_target,
                                    const Eigen::Vector2d& mean_weighted_target);
}  // namespace luhsoccer::local_planner