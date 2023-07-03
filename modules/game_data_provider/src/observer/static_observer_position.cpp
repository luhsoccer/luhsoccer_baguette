#include <cmath>

#include "observer/static_observer.hpp"

#include "config_provider/config_store_main.hpp"

#include "observer/goal_probability/goal_probability.hpp"
#include "observer/pass_probability/pass_probability.hpp"
#include "observer/utility.hpp"
#include "observer/misc/misc.hpp"

namespace luhsoccer::observer::calculation {

std::optional<double> calculateGoalProbability(const transform::Position& pos, Team team,
                                               std::shared_ptr<const transform::WorldModel> wm,
                                               const time::TimePoint time) {
    if (wm == nullptr) return std::nullopt;

    /* ----------------- BALL SHADOWS ----------------- */
    // calculate the shadows on the goalline assuming the ball as a lightsource
    auto points = goal_probability::calculateShadows(pos, team, wm, std::nullopt, time);
    if (!points.has_value()) return std::nullopt;

    // cutting the shadows to the size of the goal
    auto shadows = goal_probability::cutShadows(*points);
    // combine and cut overlapping shadows
    auto goalshadows = goal_probability::combineShadows(shadows);
    // find the biggest shadow
    // calculate the total area covered by the shadows
    // (these shaddows are now non-overlapping) (top(x) - bottom(y))
    double covered_goal_area = 0;
    for (const auto& shadow : goalshadows) {
        covered_goal_area += abs(shadow.x() - shadow.y());
    }
    // calculate how much of the goal (1m in size) is still free
    constexpr double GOAL_SIZE = 1.0;  // @todo get from World Model
    const double free_goal_area = (GOAL_SIZE - covered_goal_area);

    /* ----------------- ANGLE PROBABILITY ----------------- */
    // this function represents shooting a goal from a certain angle (goes from 1 to 0 in the range [0; PI])
    const std::optional<double> goal_from_angle_prob = goal_probability::goalFromAngleProb(pos, team, wm, time);
    if (!goal_from_angle_prob.has_value()) return std::nullopt;

    /* ----------------- DISTANCE PROBABILITY ----------------- */
    // this function represent the propability to score a goal from a certain distance
    const std::optional<double> goal_from_distance_prob = goal_probability::goalFromDistanceProb(pos, team, wm, time);
    if (!goal_from_distance_prob.has_value()) return std::nullopt;

    /* ----------------- FINAL RESULT ----------------- */
    // between 0 and 1
    return *goal_from_distance_prob * *goal_from_angle_prob * free_goal_area;
}

std::optional<Eigen::Vector2d> calculateBestGoalPoint(const transform::Position& pos, Team team,
                                                      std::shared_ptr<const transform::WorldModel> wm,
                                                      std::optional<RobotIdentifier> id_to_ignore,
                                                      const time::TimePoint time) {
    if (wm == nullptr) return std::nullopt;

    // calculate the shadows on the goalline assuming the ball as a lightsource
    auto points = goal_probability::calculateShadows(pos, team, wm, id_to_ignore, time);
    if (!points.has_value()) return std::nullopt;

    // cutting the shadows to the size of the goal
    const auto shadows = goal_probability::cutShadows(*points);

    // combine and cut overlapping shadows
    const auto goal_shadows = goal_probability::combineShadows(shadows);

    // Inverse tha shaddows
    const auto inverse_shaddows = goal_probability::invertShadows(goal_shadows);

    // find the biggest shadow
    double biggest_shadow = 0;
    double best_goal_point = 0;
    for (const Eigen::Vector2d& it : inverse_shaddows) {
        double shadows_size = abs(it.y() - it.x());
        if (shadows_size > biggest_shadow) {
            biggest_shadow = shadows_size;
            // best_goal_point is in the middle of the biggest goalshadow
            best_goal_point = it.sum() / 2.0;
        }
    }

    // if no real goal point was found just try to shoot at the middle of the goal
    constexpr double HALF_GOAL_SIZE = 0.5;
    if (!(-HALF_GOAL_SIZE <= best_goal_point && best_goal_point <= HALF_GOAL_SIZE)) {
        return std::nullopt;
    }

    // check if the window for the found goal point is reasonably sized
    const double min_size =
        config_provider::ConfigProvider::getConfigStore().observer_config.best_goal_point_min_hole_size;
    if (biggest_shadow < min_size) {
        return std::nullopt;
    }

    double x_goal_point = misc::getTeamsEnemyGoalPoint(team, *wm).x();

    return Eigen::Vector2d{x_goal_point, best_goal_point};
}

std::optional<double> calculateThreatLevel(const transform::Position& enemy_pos,
                                           std::shared_ptr<const transform::WorldModel> world_model_ptr,
                                           const time::TimePoint time) {
    if (world_model_ptr == nullptr) return std::nullopt;

    /* -------------- DISTANCE FROM GOAL -------------- */
    const auto goal_from_distance_prob =
        goal_probability::goalFromDistanceProb(enemy_pos, Team::ENEMY, world_model_ptr, time);
    if (!goal_from_distance_prob.has_value()) return std::nullopt;

    /* -------------- ANGLE-PROBABILITY -------------- */
    const auto goal_from_angle_prob =
        goal_probability::goalFromAngleProb(enemy_pos, Team::ENEMY, world_model_ptr, time);
    if (!goal_from_angle_prob.has_value()) return std::nullopt;

    /* -------------- DISTANCE TO BALL -------------- */
    const auto robot_pos = enemy_pos.getCurrentPosition(world_model_ptr, world_model_ptr->getGlobalFrame(), time);
    if (!robot_pos.has_value()) return std::nullopt;

    const auto ball_pos = transform::helper::getBallPosition(*world_model_ptr, time);
    if (!ball_pos.has_value()) return std::nullopt;

    const double distance_to_ball = utility::calculateDistance(robot_pos->translation(), *ball_pos);

    // Get Config Provider for factors
    const auto& cs = config_provider::ConfigProvider::getConfigStore();

    // to calculate the threat resulting from the ball distance,
    // use a inverse function. This will result in a value of 1 at 0m and small values at further distances
    const double inverse_factor = cs.observer_config.threat_score_inverse_ball_distance_factor;
    double distance_to_ball_threat = 1 / (distance_to_ball * inverse_factor + 1);

    /* -------------- FINAL RESULT -------------- */
    // the threat-score is calculated by a weighted sum
    const double a = cs.observer_config.threat_score_ball_distance_factor;
    const double b = cs.observer_config.threat_score_goal_distance;
    const double c = cs.observer_config.threat_score_goal_rotation;
    return (a * distance_to_ball_threat + b * *goal_from_distance_prob + c * *goal_from_angle_prob) / (a + b + c);
}

std::optional<bool> isPassLineDefended(const transform::Position& passer, const transform::Position& receiver,
                                       const Team team, std::shared_ptr<const transform::WorldModel> world_model,
                                       const time::TimePoint time) {
    /* NEWLINE */

    if (world_model == nullptr) return std::nullopt;

    // Get positions of robots
    const auto passer_affine = passer.getCurrentPosition(world_model, world_model->getGlobalFrame(), time);
    if (!passer_affine.has_value()) return std::nullopt;

    const auto receiver_affine = receiver.getCurrentPosition(world_model, world_model->getGlobalFrame(), time);
    if (!receiver_affine.has_value()) return std::nullopt;

    // Translate position of robots
    const auto passer_pos = passer_affine->translation();
    const auto receiver_pos = receiver_affine->translation();

    const auto passer_receiver_vector = receiver_pos - passer_pos;

    // The angle for when an enemy is in this angle the pass counts as blocked
    const double pass_defend_angle =
        (config_provider::ConfigProvider::getConfigStore().observer_config.pass_defended_angle) * L_PI / 180.0;

    for (const auto enemy : world_model->getVisibleRobots(time)) {
        // Only consider enemies of the passer
        if (enemy.getTeam() == team) continue;

        const auto enemy_pos = transform::helper::getPosition(transform::RobotHandle(enemy, world_model), time);
        if (!enemy_pos.has_value()) continue;

        const auto passer_defender_vector = *enemy_pos - passer_pos;

        // Skip enemies that are behind the receiver
        constexpr double ROBOT_SIZE_THRESHOLD = 0.1;
        const double distance_passer_receiver = passer_receiver_vector.norm();
        const double distance_passer_defender = passer_defender_vector.norm();
        if (distance_passer_defender > distance_passer_receiver + ROBOT_SIZE_THRESHOLD) continue;

        double angle = utility::calculateAngle(passer_receiver_vector, passer_defender_vector);

        // correct angle for the halve > PI
        angle = abs(cropAngle(angle));

        // if an enemy of the passer is in a sufficient angle to intercept the ball, count the pass as being blocked
        if (angle < pass_defend_angle) {
            return true;
        }
    }

    return false;
}

std::optional<AllyRobot::BestPassReceiver> calculateBestPassReceiver(
    const transform::Position& passer, Team team, std::shared_ptr<const transform::WorldModel> world_model,
    std::vector<RobotIdentifier> allies_to_evaluate, std::optional<RobotIdentifier> passing_robot,
    const time::TimePoint time) {
    //

    if (world_model == nullptr) return std::nullopt;

    // Used to store the current best pass receiver
    double max_pass_score = std::numeric_limits<double>::lowest();
    std::optional<RobotIdentifier> max_score_identifier = std::nullopt;

    const auto passer_affine = passer.getCurrentPosition(world_model, world_model->getGlobalFrame(), time);
    if (!passer_affine.has_value()) return std::nullopt;
    const auto passer_pos = passer_affine->translation();

    // The distance in which a robot will be assumed to be the robot of which the position was given
    constexpr double POSITION_MARGIN = 0.05;

    const auto& cs = config_provider::ConfigProvider::getConfigStore();
    const double min_dist = cs.observer_config.receiver_min_goal_dist;

    // if the allies that should be evaluated are empty all allies should be evaluated
    if (allies_to_evaluate.empty()) {
        allies_to_evaluate = world_model->getVisibleRobots();
    }

    // loop over all ally Robots (except over the robot which performs the pass)
    for (const auto& receiver_identifier : allies_to_evaluate) {
        // check handle conditions (skip Robots that are not of the same team as the given robot; skip the robot
        // which performs the pass)
        if (receiver_identifier.getTeam() != team) continue;

        const auto receiver_pos =
            transform::helper::getPosition(transform::RobotHandle(receiver_identifier, world_model), time);
        if (!receiver_pos.has_value()) continue;

        // Ignore robots which are too close to the goal
        {
            const auto dist_to_goal =
                (team == Team::ALLY)
                    ? utility::distanceToAllyGoalPoint(transform::RobotHandle(receiver_identifier, world_model))
                    : utility::distanceToEnemyGoalPoint(transform::RobotHandle(receiver_identifier, world_model));

            if (dist_to_goal.has_value()) {
                if (dist_to_goal < min_dist) continue;
            }

            // skip the passing-robot-identifier that was given (or the robot that is very close to the given position)
            if (passing_robot.has_value()) {
                if (*passing_robot == receiver_identifier) continue;
            } else {
                const auto robot_dist = utility::calculateDistance(passer_pos, *receiver_pos);
                if (robot_dist < POSITION_MARGIN) continue;
            }
        }

        // calculate the pass probability of the given robot (How likely is the pass to succeed)
        {
            const auto pass_probability = pass_probability::calculatePassProbability(
                passer, transform::Position(receiver_identifier.getFrame()), team, world_model, time);
            if (!pass_probability.has_value()) continue;

            if (*pass_probability > max_pass_score) {
                max_pass_score = *pass_probability;
                max_score_identifier = receiver_identifier;
            }
        }
    }

    // check if we have actually found a best pass receiver (should always succeed except if we have no robots)
    if (!max_score_identifier.has_value()) {
        // LOG_DEBUG(logger::Logger("observer"), "max_score_identifier had no value, BestPassReceiver not set!");
        return std::nullopt;
    }

    // return an instance of the BestPassReceiver object
    return AllyRobot::BestPassReceiver{transform::RobotHandle(*max_score_identifier, world_model), max_pass_score};
}

std::optional<double> calculatePassProbability(const transform::Position& passing_robot,
                                               const transform::Position& receiving_robot, const Team team,
                                               const std::shared_ptr<const transform::WorldModel>& world_model_ptr,
                                               const time::TimePoint time) {
    return pass_probability::calculatePassProbability(passing_robot, receiving_robot, team, world_model_ptr, time);
}

std::optional<std::pair<Eigen::Vector2d, bool>> calculateShootPoint(
    const transform::Position& robot, Team team, std::shared_ptr<const transform::WorldModel> world_model,
    const time::TimePoint time) {
    // In Meters:
    const auto field_size = misc::getFieldSize(*world_model);
    const double field_width = field_size.x();
    const double field_height = field_size.y();
    constexpr double GOAL_SIZE = 1.0;

    struct Line {
        Line(Eigen::Vector2d pos, Eigen::Vector2d dir) : pos(std::move(pos)), dir(std::move(dir)) {}
        Eigen::Vector2d pos;
        Eigen::Vector2d dir;
    };

    if (world_model == nullptr) return std::nullopt;

    // Get Robot Data
    const auto affine = robot.getCurrentPosition(world_model, world_model->getGlobalFrame(), time);
    if (!affine.has_value()) {
        return std::nullopt;
    }
    const double angle = Eigen::Rotation2Dd(affine->rotation()).angle();

    // Define a line which starts at the robot position and faces the way the robot is facing
    // d: (rx, ry) + t * (dx, dy)
    const Eigen::Vector2d pos = affine->translation();
    const Eigen::Vector2d robot_dir = utility::calculateRotationVector(angle);

    // if the robot has absolutely no horizontal direction it is lookin in, it cant hit the left or the right wall
    if (robot_dir.x() != 0.0) {
        // Define left Wall w4: (wx, wy) + r4 * (0, HEIGHT)
        Line vertical_wall({-field_width / 2, -field_height / 2}, {0, field_height});

        // if the robot is looking to the right we dont want to evaluate the left wall, but rather the right one
        /// because left_wall = right_wall + (WIDTH, 0) we just transform the left wall into a right wall
        bool is_left_wall = true;
        if (robot_dir.x() > 0) {
            is_left_wall = false;
            vertical_wall.pos.x() += field_width;
        }

        // solve system of linear equations which result when setting d = w4
        const double t = (vertical_wall.pos.x() - pos.x()) / robot_dir.x();
        const double r4 = (pos.y() + t * robot_dir.y() - vertical_wall.pos.y()) / vertical_wall.dir.y();

        // if we have already found a match take it
        if (r4 >= 0.0 && r4 <= 1.0) {
            const Eigen::Vector2d edge_intersect = pos + t * robot_dir;

            // evaluate whether the point is inside our Goal
            bool robot_points_inside_goal =
                ((team == Team::ENEMY && is_left_wall) || (team == Team::ALLY && !is_left_wall)) &&
                edge_intersect.y() >= -GOAL_SIZE / 2 && edge_intersect.y() <= GOAL_SIZE / 2;

            return std::make_pair(edge_intersect, robot_points_inside_goal);
        }
    }

    // if the robot has absolutely no vertical direction it is lookin in, it cant hit the bottom or top Wall
    if (robot_dir.y() != 0.0) {
        // Define bottom Wall w2: (wx, wy) + r2 * (WIDTH,0)
        Line horizontal_wall({-field_width / 2, -field_height / 2}, {field_width, 0});

        // if the robot is facing to the top evaluate the top wall instead
        if (robot_dir.y() > 0.0) {
            horizontal_wall.pos.y() += field_height;
        }

        // solve system of linear equations which result when setting d = w2
        const double t = (horizontal_wall.pos.y() - pos.y()) / robot_dir.y();
        const double r2 = (pos.x() + t * robot_dir.x() - horizontal_wall.pos.x()) / horizontal_wall.dir.x();

        // if we have found a match, return it
        if (r2 >= 0.0 && r2 <= 1.0) {
            const Eigen::Vector2d edge_intersect = pos + t * robot_dir;
            return std::make_pair(edge_intersect, false);
        }
    }

    // should never happen (in this case the robots LOS did not hit any wall)
    return std::nullopt;
}

std::optional<bool> isOutgoingPassDefended(const transform::Position& robot, Team team,
                                           std::shared_ptr<const transform::WorldModel> world_model,
                                           std::optional<RobotIdentifier> robot_id, const time::TimePoint time) {
    for (const auto robots : world_model->getVisibleRobots()) {
        if (robots.getTeam() != team) continue;

        if (robot_id == robots) continue;

        const auto is_defended = isPassLineDefended(robot, transform::RobotHandle(robots, world_model).getPosition(),
                                                    team, world_model, time);
        if (!is_defended.has_value()) continue;

        if (is_defended == false) {
            return false;
        }
    }

    return true;
}

double calcPassSuccessChance(const Eigen::Vector2d& start_pos, const Eigen::Vector2d& end_pos,
                             const Eigen::Vector2d& kicker_pos, const std::vector<Eigen::Vector2d>& enemies,
                             double time, double pass_power, double enemy_max_speed, double radius_scaling,
                             const util::Lut1D& pass_power_lut, const util::Lut1D& turn_time_lut,
                             const util::Lut2D& pass_time_lut) {
    return pass_probability::calcPassSuccessChance(start_pos, end_pos, kicker_pos, enemies, time, pass_power,
                                                   enemy_max_speed, radius_scaling, pass_power_lut, turn_time_lut,
                                                   pass_time_lut);
}

}  // namespace luhsoccer::observer::calculation
