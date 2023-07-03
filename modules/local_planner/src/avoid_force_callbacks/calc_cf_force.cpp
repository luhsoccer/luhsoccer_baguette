#include "calc_cf_force.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::local_planner {
Eigen::Vector2d calcCFForce(const Eigen::Vector2d& vec_to_feature, const Eigen::Vector2d& relative_velocity,
                            bool magentic_field_upwards) {
    Eigen::Vector3d robot_to_feature_3d = {vec_to_feature.x(), vec_to_feature.y(), 0.0};
    Eigen::Vector3d relative_velocity_3d = {relative_velocity.x(), relative_velocity.y(), 0.0};

    if (robot_to_feature_3d.dot(relative_velocity_3d) <= 0.0) {
        return Eigen::Vector2d::Zero();
    }

    Eigen::Vector3d magentic_field = Eigen::Vector3d::UnitZ();
    if (!magentic_field_upwards) magentic_field *= -1;

    // cf force
    Eigen::Vector3d electric_current = robot_to_feature_3d.cross(magentic_field);

    Eigen::Vector3d triple_product = relative_velocity_3d.cross(electric_current.cross(relative_velocity_3d));

    Eigen::Vector3d cf_force = Eigen::Vector3d::Zero();
    if (robot_to_feature_3d.norm() != 0.0) cf_force = triple_product / robot_to_feature_3d.squaredNorm();

    return cf_force.head(2);
}

double calcTargetRelaxationFactors(const std::optional<Eigen::Vector2d>& min_obstacle_vec,
                                   const Eigen::Vector2d& max_target_vec) {
    if (!min_obstacle_vec.has_value()) return 1.0;
    double w1 = 1.0 - std::exp(-min_obstacle_vec->norm() /
                               (localPlannerConfig().robot_radius * localPlannerConfig().step_drive_w1_alpha));
    double w2 =
        1.0 - (max_target_vec.dot(min_obstacle_vec.value()) / (max_target_vec.norm() * min_obstacle_vec->norm()));

    return std::max(w1 * w2, localPlannerConfig().setp_drive_min_relaxation_factor.val());
};

}  // namespace luhsoccer::local_planner