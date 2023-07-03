
#include "logger/logger.hpp"
#include "transform_helper/world_model_helper.hpp"
#include "observer/goal_probability/goal_probability.hpp"
#include "observer/misc/misc.hpp"

#include <cmath>
#include "observer/utility.hpp"
#include "utils/utils.hpp"
#include "config_provider/config_store_main.hpp"
#include <algorithm>

namespace luhsoccer::observer::calculation::goal_probability {

[[nodiscard]] std::optional<double> distanceToRobotsTargetGoal(const transform::RobotHandle& handle,
                                                               const time::TimePoint time) {
    return handle.isEnemy() ? utility::distanceToAllyGoalPoint(handle, time)
                            : utility::distanceToEnemyGoalPoint(handle, time);
}

std::optional<std::vector<Eigen::Vector2d>> calculateShadows(const transform::Position& ball_position, Team team,
                                                             std::shared_ptr<const transform::WorldModel> wm,
                                                             std::optional<RobotIdentifier> id_to_ignore,
                                                             const time::TimePoint time) {
    constexpr double ROBOT_RADIUS = 0.09;

    if (wm == nullptr) return std::nullopt;

    const double x_goal_point = misc::getTeamsEnemyGoalPoint(team, *wm).x();

    // Set the corresponding goalpoint for ally Team

    const auto ball_pos_affine = ball_position.getCurrentPosition(wm, wm->getGlobalFrame(), time);
    if (!ball_pos_affine.has_value()) return std::nullopt;

    const auto ball_pos = ball_pos_affine->translation();

    const auto goal_ball_distance = utility::calculateDistance(ball_pos, Eigen::Vector2d{x_goal_point, 0});

    // to save the shadows on the goalline, the x part of each vector is the top point of the shadow, while the y
    // component is the bottom
    std::vector<Eigen::Vector2d> points;

    // calculate the shaddow on the goal assuming the ball is a light source and all enemies are objects
    for (const auto identifier : wm->getVisibleRobots(time)) {
        // if a robot that should be ignore is given, skip it
        if (identifier == id_to_ignore) continue;

        const auto pos_other = transform::helper::getPosition(transform::RobotHandle(identifier, wm), time);
        if (!pos_other.has_value()) continue;

        const auto goal_distance_all_robots =
            (team == Team::ALLY) ? utility::distanceToEnemyGoalPoint(transform::RobotHandle(identifier, wm), time)
                                 : utility::distanceToAllyGoalPoint(transform::RobotHandle(identifier, wm), time);

        if (!goal_distance_all_robots.has_value()) continue;

        // A Robot is only evaluated if it is closer to the goal
        // than the ball (minus a threshold so that the shooting robot wont be counted)
        constexpr double IGNORE_THRESHOLD = 0.1;
        if (goal_distance_all_robots > goal_ball_distance - IGNORE_THRESHOLD) continue;

        // position=startingposition(pos)+t*directionvector
        // @todo my has to be improved (better position inclusion)
        const Eigen::Vector2d defending_robot_up = {pos_other->x(), pos_other->y() + ROBOT_RADIUS};
        const Eigen::Vector2d defending_robot_down = {pos_other->x(), pos_other->y() - ROBOT_RADIUS};

        // calculate factors for the equation ru = g and rd = g with
        // g: vector on the goal-line: (x_goal_point)+ t * (0, -1)
        // ru: vector from the ball to the upper point on the robot
        // rd: vector from the ball to the lower point on the robot
        const double up_factor = (x_goal_point - ball_pos.x()) / (defending_robot_up.x() - ball_pos.x());
        const double down_factor = (x_goal_point - ball_pos.x()) / (defending_robot_down.x() - ball_pos.x());

        const double up_point_y = ball_pos.y() + up_factor * (defending_robot_up.y() - ball_pos.y());
        const double down_point_y = ball_pos.y() + down_factor * (defending_robot_down.y() - ball_pos.y());
        points.emplace_back(up_point_y, down_point_y);
    }
    return points;
}

std::vector<Eigen::Vector2d> cutShadows(const std::vector<Eigen::Vector2d>& points) {
    constexpr double HALF_GOAL_SIZE = 0.5;

    std::vector<Eigen::Vector2d> shadows;
    for (const auto& point : points) {
        if (point.x() > HALF_GOAL_SIZE) {
            if (point.y() > HALF_GOAL_SIZE) {
                continue;
            }

            if (point.y() < -HALF_GOAL_SIZE) {
                shadows.emplace_back(HALF_GOAL_SIZE, -HALF_GOAL_SIZE);
            } else {
                shadows.emplace_back(HALF_GOAL_SIZE, point.y());
            }

            continue;
        }

        if (point.x() < -HALF_GOAL_SIZE) continue;

        if (point.y() < -HALF_GOAL_SIZE) {
            shadows.emplace_back(point.x(), -HALF_GOAL_SIZE);
        } else {
            shadows.push_back(point);
        }
    }

    return shadows;
}

std::vector<Eigen::Vector2d> combineShadows(std::vector<Eigen::Vector2d> shadows) {
    // Combine all shadows that overlap to a bigger shaddow
    for (size_t i = 0; i < shadows.size(); i++) {
        for (size_t j = i + 1; j < shadows.size(); j++) {
            // skip shaddows that have already been processed
            if (shadows[i].x() == -1 || shadows[j].x() == -1) continue;

            // case 2: j.x is between i.x and i.y
            // case 3: j.y is between i.x and i.y
            if ((shadows[i].y() < shadows[j].x() && shadows[j].x() < shadows[i].x()) ||
                (shadows[i].y() < shadows[j].y() && shadows[j].y() < shadows[i].x())) {
                shadows[i].x() = std::max(shadows[j].x(), shadows[i].x());
                shadows[i].y() = std::min(shadows[j].y(), shadows[i].y());
                shadows[j] = {-1, -1};
            }
        }
    }

    // remove all elements that are {-1, -1}
    std::vector<Eigen::Vector2d> filtered_shadows;
    std::copy_if(shadows.begin(), shadows.end(), std::back_inserter(filtered_shadows),
                 [](const Eigen::Vector2d& vec) -> bool { return !(vec.x() == -1 && vec.y() == -1); });

    return filtered_shadows;
}

std::vector<Eigen::Vector2d> invertShadows(const std::vector<Eigen::Vector2d>& shadows) {
    constexpr double HALF_GOAL_SIZE = 0.5;  // @todo get from world model

    std::vector<Eigen::Vector2d> inverse_shadows;

    for (const auto& vec : shadows) {
        // find y that is closest to x but still bigger
        double closest_y = HALF_GOAL_SIZE;
        bool found_change = false;
        for (const auto& inner_vec : shadows) {
            if (inner_vec.y() > vec.x() && inner_vec.y() < closest_y) {
                closest_y = inner_vec.y();
                found_change = true;
            }
        }
        // if none was found  set it to the goal height
        if (!found_change) {
            closest_y = HALF_GOAL_SIZE;
        }

        if (closest_y != vec.x()) {
            inverse_shadows.emplace_back(closest_y, vec.x());
        }
    }

    Eigen::Vector2d bottom = {-HALF_GOAL_SIZE, 0};
    // find y that is closest to x but still bigger
    double closest_y = HALF_GOAL_SIZE;
    bool found_change = false;
    for (const auto& inner_vec : shadows) {
        if (inner_vec.y() >= bottom.x() && inner_vec.y() < closest_y) {
            closest_y = inner_vec.y();
            found_change = true;
        }
    }
    // if none was found  set it to the goal height
    if (!found_change) {
        closest_y = HALF_GOAL_SIZE;
    }

    // if the last step resulted in a 0-width-shaddow dont insert it
    constexpr double MARGIN = 0.0001;
    if (!(closest_y - MARGIN < bottom.x() && bottom.x() < closest_y + MARGIN)) {
        inverse_shadows.emplace_back(closest_y, bottom.x());
    }

    return inverse_shadows;
}

std::optional<double> goalFromAngleProb(const transform::Position& ball_position, Team team,
                                        std::shared_ptr<const transform::WorldModel> wm, const time::TimePoint time) {
    if (wm == nullptr) return std::nullopt;

    // Set the corresponding goalpoint
    const double x_goal_point = misc::getTeamsEnemyGoalPoint(team, *wm).x();

    const auto ball_pos_affine = ball_position.getCurrentPosition(wm, wm->getGlobalFrame(), time);
    if (!ball_pos_affine.has_value()) return std::nullopt;
    const auto pos = ball_pos_affine->translation();

    const Eigen::Vector2d pos_to_goal_point = Eigen::Vector2d{x_goal_point, 0} - pos;
    const Eigen::Vector2d goal_point_to_x = (team == Team::ALLY) ? Eigen::Vector2d{1, 0} : Eigen::Vector2d{-1, 0};

    double angle = utility::calculateAngle(pos_to_goal_point, goal_point_to_x);

    // get angle in range [0; PI]
    angle = abs(cropAngle(angle));

    // this function represents shooting a goal from a certain angle (goes from 1 to ~0 in the range [0; PI])
    const double halve_point_angle =
        config_provider::ConfigProvider::getConfigStore().observer_config.goal_prob_goal_angle_halve_point;
    const double goal_from_angle_prob = 1 / (angle / halve_point_angle + 1);
    return goal_from_angle_prob;
}

std::optional<double> goalFromDistanceProb(const transform::Position& ball_position, Team team,
                                           std::shared_ptr<const transform::WorldModel> wm,
                                           const time::TimePoint time) {
    if (wm == nullptr) return std::nullopt;

    // Set the corresponding goalpoint
    const double x_goal_point = misc::getTeamsEnemyGoalPoint(team, *wm).x();

    const auto ball_pos_affine = ball_position.getCurrentPosition(wm, wm->getGlobalFrame(), time);
    if (!ball_pos_affine.has_value()) return std::nullopt;
    const auto pos = ball_pos_affine->translation();

    const double goal_distance = (pos - Eigen::Vector2d(x_goal_point, 0)).norm();

    // The point at which the dropoff will occur
    const double dropoff_point =
        config_provider::ConfigProvider::getConfigStore().observer_config.goal_prob_distance_dropoff_point;

    // A Function which goes fairly steady from 0 to ~2 and then starts dropping of (steep dropoff at ~dropoff_point)
    // capped between 0 and 1
    return 1 - 1 / (1 + exp(-goal_distance + dropoff_point));
}

}  // namespace luhsoccer::observer::calculation::goal_probability