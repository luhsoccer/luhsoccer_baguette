#pragma once

#include "local_planner/avoidance_manager_unit.hpp"

namespace luhsoccer::local_planner {

class HaddadinDecision : public AbstractAvoidanceManagerUnit {
   public:
    std::vector<bool> getRotationVectors(const std::vector<std::shared_ptr<const AbstractCFObstacle>>& features,
                                         const Eigen::Vector2d& goal_vec,
                                         const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                                         const RobotIdentifier& robot, time::TimePoint time) override;
};

}  // namespace luhsoccer::local_planner