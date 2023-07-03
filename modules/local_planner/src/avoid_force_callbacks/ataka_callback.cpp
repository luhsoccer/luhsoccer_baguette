#include "ataka_callback.hpp"

#include "local_planner/skills/abstract_feature.hpp"

#include "local_planner/avoidance_manager.hpp"

namespace luhsoccer::local_planner {
AvoidForceResult getAtakaForce(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                               const RobotIdentifier& robot, AvoidanceManager& /*am*/, const time::TimePoint time,
                               const std::vector<std::shared_ptr<const AbstractCFObstacle>>& obstacles,
                               const Eigen::Vector2d& target_force, const Eigen::Vector2d& /*max_target*/,
                               const Eigen::Vector2d& /*mean_weighted_target*/) {
    AvoidForceResult result;

    Eigen::Vector2d ataka_force = Eigen::Vector2d::Zero();
    // std::map<size_t, bool> rotation_vector_map;

    for (const auto& cf_feature : obstacles) {
        auto res = cf_feature->getVecAndVelocity(wm, td, robot, time);

        // rotation_vector_map[cf_feature->getUid()] = rotation_vectors[i];
        if (res.has_value() && res->first.norm() < cf_feature->getInfluenceDistance(wm, td)) {
            Eigen::Vector3d electric_current = {0.0, 0.0, 0.0};
            Eigen::Vector3d vel_3d = {res->second.x(), res->second.y(), 0.0};
            electric_current.head(2) =
                res->second - (res->second.transpose() * res->first) * res->first / std::pow(res->first.norm(), 2);
            Eigen::Vector3d triple_product = vel_3d.normalized().cross(electric_current.cross(vel_3d.normalized()));

            ataka_force += triple_product.head(2) / res->first.squaredNorm() * cf_feature->getWeight(wm, td);

            constexpr double SMALL_VALUE = 0.00001;
            if (res->first.norm() < cf_feature->getCriticalDistance(wm, td) &&
                cf_feature->getWeight(wm, td) > SMALL_VALUE) {
                result.critical_vectors.emplace_back(res->first, std::nullopt);
            }
        }
    }

    result.total_force = target_force + ataka_force * localPlannerConfig().feature_robot_obstacle_k_ataka.val();

    return result;
}

}  // namespace luhsoccer::local_planner