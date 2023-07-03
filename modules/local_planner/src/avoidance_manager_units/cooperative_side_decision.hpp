
#pragma once
#include "local_planner/avoidance_manager_unit.hpp"

namespace luhsoccer::local_planner {

class CooperativeSideDecision : public AbstractAvoidanceManagerUnit {
   public:
    std::vector<bool> getRotationVectors(const std::vector<std::shared_ptr<const AbstractCFObstacle>>& features,
                                         const Eigen::Vector2d& goal_vec,
                                         const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                                         const RobotIdentifier& robot, time::TimePoint time) override;

   private:
    struct RotationVectorState {
        bool self_defined;
        bool upwards;
    };
    struct RotationVectorData {
        std::unordered_map<RobotIdentifier, RotationVectorState> rotation_vectors;
        time::TimePoint time;
        std::shared_mutex mtx;
    };
    std::unordered_map<RobotIdentifier, RotationVectorData> rotation_vectors;
    std::shared_mutex rotation_vectors_mtx;

    const time::Duration vector_timeout_duration{0.1};
};
};  // namespace luhsoccer::local_planner