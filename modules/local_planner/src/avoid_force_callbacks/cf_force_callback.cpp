#include "cf_force_callback.hpp"

#include "local_planner/skills/abstract_feature.hpp"

#include "local_planner/avoidance_manager.hpp"

#include "calc_cf_force.hpp"
namespace luhsoccer::local_planner {
AvoidForceResult getCFForce(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                            const RobotIdentifier& robot, AvoidanceManager& am, const time::TimePoint time,
                            const std::vector<std::shared_ptr<const AbstractCFObstacle>>& obstacles,
                            const Eigen::Vector2d& target_force, const Eigen::Vector2d& max_target,
                            const Eigen::Vector2d& mean_weighted_target) {
    auto rotation_vectors = am.getRotationVectors(obstacles, mean_weighted_target, wm, td, robot, time);
    AvoidForceResult result;

    if (rotation_vectors.size() != obstacles.size())
        throw std::range_error(
            "The number of rotation vectors provided by the avoidance manager should equal the "
            "number of rotation features.");

    Eigen::Vector2d cf_feature_force = Eigen::Vector2d::Zero();
    int i = 0;
    // std::map<size_t, bool> rotation_vector_map;
    std::optional<Eigen::Vector2d> vec_to_closest_obstacle;
    constexpr double SMALL_VALUE = 0.00001;

    for (const auto& cf_feature : obstacles) {
        auto res = cf_feature->getVecAndVelocity(wm, td, robot, time);

        // rotation_vector_map[cf_feature->getUid()] = rotation_vectors[i];
        if (res.has_value() &&
            res->first.norm() < std::max(std::min(cf_feature->getInfluenceDistance(wm, td), max_target.norm()),
                                         localPlannerConfig().robot_radius.val()) &&
            cf_feature->getWeight(wm, td) > SMALL_VALUE) {
            auto cf_force = calcCFForce(res->first, res->second, rotation_vectors[i]);
            result.magnetic_field_vec_map[cf_feature->getUid()] = rotation_vectors[i];
            cf_feature_force +=
                cf_force * localPlannerConfig().feature_robot_obstacle_k_cf.val() * cf_feature->getWeight(wm, td);
            // determine closes cf obstacle vector
            if (!vec_to_closest_obstacle.has_value() || res->first.norm() < vec_to_closest_obstacle->norm())
                vec_to_closest_obstacle = res->second;

            if (res->first.norm() < cf_feature->getCriticalDistance(wm, td)) {
                result.critical_vectors.emplace_back(res->first, rotation_vectors[i]);
            }
            result.influence_map[cf_feature->getUid()] = cf_force.norm() > SMALL_VALUE;
        } else {
            result.influence_map[cf_feature->getUid()] = false;
        }
        i++;
    }
    double goal_relaxation_factors = calcTargetRelaxationFactors(vec_to_closest_obstacle, max_target);

    result.total_force = target_force * goal_relaxation_factors + cf_feature_force;

    return result;
}
}  // namespace luhsoccer::local_planner