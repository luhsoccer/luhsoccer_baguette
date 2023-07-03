#pragma once

#include "local_planner/avoidance_manager_unit.hpp"
#include "local_planner/simulation_manager.hpp"

namespace luhsoccer::local_planner {

class MultiAgent : public AbstractAvoidanceManagerUnit {
   public:
    MultiAgent(SimulationManager& simulation_manager);

    std::vector<bool> getRotationVectors(const std::vector<std::shared_ptr<const AbstractCFObstacle>>& features,
                                         const Eigen::Vector2d& goal_vec,
                                         const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                                         const RobotIdentifier& robot, time::TimePoint time) override;

   private:
    SimulationManager& simulation_manager;
};

}  // namespace luhsoccer::local_planner