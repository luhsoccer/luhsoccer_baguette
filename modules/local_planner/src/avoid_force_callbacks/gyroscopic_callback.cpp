#include "gyroscopic_callback.hpp"

#include "local_planner/skills/abstract_feature.hpp"

#include "local_planner/avoidance_manager.hpp"

namespace luhsoccer::local_planner {

std::optional<Eigen::Vector2d> calcGyroscopicForce(const std::shared_ptr<const transform::WorldModel>& wm,
                                                   const TaskData& /*td*/, const RobotIdentifier& robot,
                                                   const time::TimePoint time, const Eigen::Vector2d& target_force,
                                                   const Eigen::Vector2d& obstacle_vec,
                                                   const Eigen::Vector2d& vec_to_closest_obstacle) {
    Eigen::Vector3d target_force_3d = {target_force.x(), target_force.y(), 0.0};
    Eigen::Vector3d obstacle_vector_3d = {obstacle_vec.x(), obstacle_vec.y(), 0.0};
    Eigen::Vector3d closest_obstacle_vec_3d = {vec_to_closest_obstacle.x(), vec_to_closest_obstacle.y(), 0.0};

    auto robot_vel = transform::Position(robot.getFrame()).getVelocity(wm, "", "", time);
    if (!robot_vel.has_value()) return std::nullopt;

    Eigen::Vector3d robot_vel_3d = {robot_vel->x(), robot_vel->y(), 0.0};
    // NOLINTBEGIN (cppcoreguidelines-avoid-magic-numbers)
    //  breaking force
    Eigen::Vector3d breaking_force = {0.0, 0.0, 0.0};

    double v = 0.0;  // like in Sabbatinine paper under eq 18
    if (obstacle_vec.norm() >= 0.01) {
        v = closest_obstacle_vec_3d.dot(robot_vel_3d);
    }
    constexpr double SIGMA = 0.001;
    if (v > 0.001) {
        breaking_force << -1 * SIGMA * (std::fabs(v) + std::exp(-1.0 * std::fabs(v))) *
                              closest_obstacle_vec_3d.normalized();
    }
    // gyroscopic force

    // potential field gradient--> equal to u_i_t in sabbattini paper
    Eigen::Vector3d pf_grad = -1 * obstacle_vector_3d + target_force_3d;

    if (v > 0.001) {
        // equal to eq 30
        Eigen::Vector3d phi = {0.0, 0.0, 0.0};

        if (robot_vel_3d.norm() >= 0.1) {
            phi = robot_vel_3d;
        } else {
            phi = closest_obstacle_vec_3d;
        }

        // equal to eq 31
        Eigen::Vector3d xi = pf_grad - (pf_grad.dot(phi) * phi);

        double alignment = pf_grad.normalized().dot(closest_obstacle_vec_3d.normalized());

        // equal to eq 32 and 69
        // chose that to compensate random oscilation at the goal and local minima
        Eigen::Vector3d w = {0.0, 0.0, 0.0};
        if (xi.norm() != 0 && std::abs(alignment) < 0.9) {
            w = xi;
        } else {
            Eigen::Vector3d random_pertubation;
            double rand_x = rand() % 10 - 5;
            double rand_y = rand() % 40 - 20;
            double rand_z = rand() % 40 - 20;
            random_pertubation << rand_x, rand_y, rand_z;  //???????????
            // random_pertubation << 0.0, 5.0, 0.0; //??????????????
            // std::cout << random_pertubation << std::endl;
            // std::cout << "------------------ " << std::endl;

            // std::cout << "before pf grad: " << std::endl << pf_grad << std::endl;

            pf_grad = pf_grad + random_pertubation;
            // std::cout << "after pf grad: " << std::endl << pf_grad << std::endl;

            w = pf_grad - (pf_grad.dot(phi) * phi);
        }

        // equal to eq 33
        if (pf_grad.norm() >= 0.1) {
            w.normalize();
            return localPlannerConfig().feature_robot_obstacle_k_gyro.val() * (w.head(2) + breaking_force.head(2));
        } else {
            return localPlannerConfig().feature_robot_obstacle_k_gyro.val() * breaking_force.head(2);
        }
    }
    // NOLINTEND
    return Eigen::Vector2d::Zero();
}

AvoidForceResult getGyroscopicForce(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                                    const RobotIdentifier& robot, AvoidanceManager& /*am*/, const time::TimePoint time,
                                    const std::vector<std::shared_ptr<const AbstractCFObstacle>>& obstacles,
                                    const Eigen::Vector2d& target_force, const Eigen::Vector2d& /*max_target*/,
                                    const Eigen::Vector2d& mean_weighted_target) {
    AvoidForceResult result;

    Eigen::Vector2d obstacle_vector = Eigen::Vector2d::Zero();
    // std::map<size_t, bool> rotation_vector_map;
    std::optional<Eigen::Vector2d> vec_to_closest_obstacle;

    for (const auto& cf_feature : obstacles) {
        auto res = cf_feature->getVecAndVelocity(wm, td, robot, time);

        // rotation_vector_map[cf_feature->getUid()] = rotation_vectors[i];
        if (res.has_value() && res->first.norm() < cf_feature->getInfluenceDistance(wm, td)) {
            if (res->first.dot(res->second) > 0.0) {
                obstacle_vector += res->first;
            }
            constexpr double SMALL_VALUE = 0.00001;
            if (res->first.norm() < cf_feature->getCriticalDistance(wm, td) &&
                cf_feature->getWeight(wm, td) > SMALL_VALUE) {
                result.critical_vectors.emplace_back(res->first, std::nullopt);
            }
            if (!vec_to_closest_obstacle.has_value() || res->second.norm() < vec_to_closest_obstacle->norm())
                vec_to_closest_obstacle = res->first;
        }
    }
    std::optional<Eigen::Vector2d> cf_feature_force_opt;
    if (vec_to_closest_obstacle.has_value()) {
        cf_feature_force_opt = calcGyroscopicForce(wm, td, robot, time, mean_weighted_target, obstacle_vector,
                                                   vec_to_closest_obstacle.value());
    }
    result.total_force = target_force;
    if (cf_feature_force_opt.has_value()) result.total_force += cf_feature_force_opt.value();
    return result;
}

}  // namespace luhsoccer::local_planner