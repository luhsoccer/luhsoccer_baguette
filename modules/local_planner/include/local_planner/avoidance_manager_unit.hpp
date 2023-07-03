#pragma once
#include "local_planner/skills/abstract_feature.hpp"

namespace luhsoccer::local_planner {

struct AvoidForceResult {
    Eigen::Vector2d total_force;
    std::vector<std::pair<Eigen::Vector2d, std::optional<bool>>> critical_vectors;
    std::map<size_t, bool> influence_map;
    std::map<size_t, bool> magnetic_field_vec_map;
};

class AbstractAvoidanceManagerUnit {
   public:
    AbstractAvoidanceManagerUnit() = default;
    AbstractAvoidanceManagerUnit(const AbstractAvoidanceManagerUnit&) = default;
    AbstractAvoidanceManagerUnit(AbstractAvoidanceManagerUnit&&) = default;
    AbstractAvoidanceManagerUnit& operator=(const AbstractAvoidanceManagerUnit&) = default;
    AbstractAvoidanceManagerUnit& operator=(AbstractAvoidanceManagerUnit&&) = default;
    virtual ~AbstractAvoidanceManagerUnit() = default;

    virtual std::vector<bool> getRotationVectors(const std::vector<std::shared_ptr<const AbstractCFObstacle>>& features,
                                                 const Eigen::Vector2d& goal_vec,
                                                 const std::shared_ptr<const transform::WorldModel>& wm,
                                                 const TaskData& td, const RobotIdentifier& robot,
                                                 time::TimePoint time) = 0;
};
}  // namespace luhsoccer::local_planner