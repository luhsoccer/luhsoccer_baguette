
#include <Eigen/Geometry>
#include "observer/utility.hpp"

#include "transform/world_model.hpp"
#include "utils/utils.hpp"
#include "core/common_types.hpp"

namespace luhsoccer::observer::calculation::misc {

std::pair<bool, double> isInsideAngle(const Eigen::Vector2d& ball_robot_vec, const Eigen::Vector2d& ball_vel,
                                      const double max_robot_vel, const double alpha_factor) {
    const double alpha = (std::atan(max_robot_vel / ball_vel.norm()) * 180.0 / L_PI) * alpha_factor;
    const double robot_ball_angle = utility::calculateAngle(ball_robot_vec, ball_vel);
    const double robot_ball_angle_cropped = abs(cropAngle(robot_ball_angle));
    const double robot_ball_angle_deg = robot_ball_angle_cropped * 180.0 / L_PI;

    return {robot_ball_angle_deg < alpha, alpha};
}

Eigen::Vector2d getTeamsEnemyGoalPoint(const Team team, const transform::WorldModel& wm) {
    const double fallback_x_goal_point_enemy = wm.getFieldData().size.x() / 2;
    const double fallback_x_goal_point_ally = -wm.getFieldData().size.x() / 2;

    const auto gp = (team == Team::ALLY) ? wm.getTransform(transform::field::GOAL_ENEMY_CENTER)
                                         : wm.getTransform(transform::field::GOAL_ALLY_CENTER);

    if (!gp.has_value()) {
        return (team == Team::ALLY) ? Eigen::Vector2d{fallback_x_goal_point_enemy, 0}
                                    : Eigen::Vector2d{fallback_x_goal_point_ally, 0};
    }

    return gp->transform.translation();
}

}  // namespace luhsoccer::observer::calculation::misc