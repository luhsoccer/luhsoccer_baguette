
#include <Eigen/Geometry>
#include "observer/utility.hpp"

#include "transform/world_model.hpp"
#include "utils/utils.hpp"
#include "common_types.hpp"

namespace luhsoccer::observer::calculation::misc {

bool isInsideAngle(const Eigen::Vector2d& ball_robot_vec, const Eigen::Vector2d& ball_vel, const double max_robot_vel,
                   const double alpha_factor) {
    const double alpha = (std::atan(max_robot_vel / ball_vel.norm()) * 180.0 / L_PI) * alpha_factor;
    const double robot_ball_angle = utility::calculateAngle(ball_robot_vec, ball_vel);
    const double robot_ball_angle_cropped = abs(cropAngle(robot_ball_angle));
    const double robot_ball_angle_deg = robot_ball_angle_cropped * 180.0 / L_PI;

    return robot_ball_angle_deg < alpha;
}

Eigen::Vector2d getTeamsEnemyGoalPoint(const Team team, const transform::WorldModel& wm) {
    constexpr double FALLBACK_X_GOAL_POINT_ENEMY = 4.5;
    constexpr double FALLBACK_X_GOAL_POINT_ALLY = -4.5;

    const auto gp = (team == Team::ALLY) ? wm.getTransform(transform::field::GOAL_ENEMY_CENTER)
                                         : wm.getTransform(transform::field::GOAL_ALLY_CENTER);

    if (!gp.has_value()) {
        return (team == Team::ALLY) ? Eigen::Vector2d{FALLBACK_X_GOAL_POINT_ENEMY, 0}
                                    : Eigen::Vector2d{FALLBACK_X_GOAL_POINT_ALLY, 0};
    }

    return gp->transform.translation();
}

Eigen::Vector2d getFieldSize(const transform::WorldModel& wm) {
    const auto top_right = wm.getTransform(transform::field::CORNER_ENEMY_LEFT);
    const auto bottom_left = wm.getTransform(transform::field::CORNER_ALLY_RIGHT);

    if (!top_right.has_value() || !bottom_left.has_value()) {
        constexpr double FALLBACK_FIELD_WIDTH = 9.0;
        constexpr double FALLBACK_FIELD_HEIGHT = 6.0;
        return Eigen::Vector2d{FALLBACK_FIELD_WIDTH, FALLBACK_FIELD_HEIGHT};
    }

    const auto tr = top_right->transform.translation();
    const auto bl = bottom_left->transform.translation();

    double field_width = tr.x() - bl.x();
    double field_height = tr.y() - bl.y();
    return Eigen::Vector2d{field_width, field_height};
}

}  // namespace luhsoccer::observer::calculation::misc