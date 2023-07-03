#include "apf_callback.hpp"

#include "local_planner/skills/abstract_feature.hpp"

#include "local_planner/avoidance_manager.hpp"

namespace luhsoccer::local_planner {
AvoidForceResult getAPFForce(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                             const RobotIdentifier& robot, AvoidanceManager& /*am*/, const time::TimePoint time,
                             const std::vector<std::shared_ptr<const AbstractCFObstacle>>& obstacles,
                             const Eigen::Vector2d& target_force, const Eigen::Vector2d& /*max_target*/,
                             const Eigen::Vector2d& /*mean_weighted_target*/) {
    AvoidForceResult result;

    Eigen::Vector2d apf_force = Eigen::Vector2d::Zero();
    // std::map<size_t, bool> rotation_vector_map;

    for (const auto& cf_feature : obstacles) {
        auto res = cf_feature->getVecAndVelocity(wm, td, robot, time);

        // rotation_vector_map[cf_feature->getUid()] = rotation_vectors[i];
        if (res.has_value() && res->first.norm() < cf_feature->getInfluenceDistance(wm, td)) {
            apf_force += (1.0 / res->first.norm() - 1.0 / cf_feature->getInfluenceDistance(wm, td)) /
                         res->first.squaredNorm() * -res->first *
                         localPlannerConfig().feature_robot_obstacle_k_apf.val();

            constexpr double SMALL_VALUE = 0.00001;
            if (res->first.norm() < cf_feature->getCriticalDistance(wm, td) &&
                cf_feature->getWeight(wm, td) > SMALL_VALUE) {
                result.critical_vectors.emplace_back(res->first, std::nullopt);
            }
        }
    }

    result.total_force = target_force + apf_force;

    return result;
}

}  // namespace luhsoccer::local_planner