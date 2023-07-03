
#include "utils/utils.hpp"
#include "config_provider/config_store_main.hpp"

#include "observer/misc/misc.hpp"
#include "transform_helper/world_model_helper.hpp"
#include "observer/pass_probability/pass_probability.hpp"
#include "observer/utility.hpp"

namespace {

double squaredLineSegmentLength(const Eigen::Vector2d& start, const Eigen::Vector2d& end,
                                const Eigen::Vector2d& circle_mid, double radius) {
    double squared_radius = std::pow(radius, 2.0);
    Eigen::Vector2d line_vector = end - start;
    Eigen::Vector2d point_vector = circle_mid - start;

    double projection = point_vector.dot(line_vector) / line_vector.squaredNorm();

    Eigen::Vector2d closest_point = {0.0, 0.0};

    if (projection < 0.0) {
        closest_point = start;
    } else if (projection > 1.0) {
        closest_point = end;
    } else {
        closest_point = start + projection * line_vector;
    }

    Eigen::Vector2d mid_to_closest = circle_mid - closest_point;
    double squared_distance = mid_to_closest.squaredNorm();

    if (squared_distance < 0.02) {
        return radius;
    }

    if (squared_distance > squared_radius) {
        return 0.0;
    }

    return squared_radius - squared_distance;
}

}  // namespace

namespace luhsoccer::observer::calculation::pass_probability {

double calcPassSuccessChance(const Eigen::Vector2d& start_pos, const Eigen::Vector2d& end_pos,
                             const Eigen::Vector2d& kicker_pos, const std::vector<Eigen::Vector2d>& enemies,
                             double time, double pass_power, double enemy_max_speed, double radius_scaling,
                             const util::Lut1D& pass_power_lut, const util::Lut1D& turn_time_lut,
                             const util::Lut2D& pass_time_lut) {
    Eigen::Vector2d start_to_end = end_pos - start_pos;

    if (pass_power < 0) {
        double dist_to_target = start_to_end.norm();
        pass_power = pass_power_lut.interpolate(dist_to_target);
    }

    // calculate the required Turn Angle
    Eigen::Vector2d start_to_kicker = start_pos - kicker_pos;
    double angle_ball_robot = std::atan2(start_to_kicker.y(), start_to_kicker.x());
    double angle_ball_target = std::atan2(start_to_end.y(), start_to_end.x());
    double turn_angle = angle_ball_target - angle_ball_robot;

    if (turn_angle > L_PI) {
        turn_angle -= 2 * L_PI;
    } else if (turn_angle < -L_PI) {
        turn_angle += 2 * L_PI;
    }

    // Calculate the time needed to execute the shot. This assumes that the robot already has ball possession
    turn_angle = std::abs(turn_angle);

    double turn_time = turn_time_lut.interpolate(turn_angle * 180.0 / L_PI);
    double time_until_shot = turn_time + time;

    if (time_until_shot < 0) {
        time_until_shot = 0.0;
    }

    // Calculate the interception chances based on the traveldistance from ball through the enemy action spheres
    // the action radius depends on the agents max. speed and the available time for interception

    double max_interception_chance = 0.0;

    for (const auto& enemy_pos : enemies) {
        Eigen::Vector2d enemy_to_ball = start_pos - enemy_pos;
        double dist_enemy_ball = enemy_to_ball.norm();
        double time_for_interception = pass_time_lut.interpolate(dist_enemy_ball, pass_power);

        double action_radius = enemy_max_speed * (time_until_shot + time_for_interception) / radius_scaling;

        if (action_radius < 0.01) {
            action_radius = 0.01;
        }

        double intercept_point_to_robot = squaredLineSegmentLength(start_pos, end_pos, enemy_pos, action_radius);
        double interception_chance = intercept_point_to_robot / action_radius;

        if (interception_chance > max_interception_chance) {
            max_interception_chance = interception_chance;
        }
    }

    return 1 - max_interception_chance;
}

std::optional<double> calculatePassProbability(const transform::RobotHandle& passing_robot,
                                               const RobotIdentifier& receiving_robot,
                                               const std::shared_ptr<const transform::WorldModel>& world_model_ptr,
                                               const time::TimePoint time) {
    return calculatePassProbability(passing_robot.getPosition(), transform::Position(receiving_robot.getFrame()),
                                    passing_robot.getTeam(), world_model_ptr, time);
}

std::optional<double> calculatePassProbability(const transform::Position& passing_robot,
                                               const transform::Position& receiving_robot, const Team team,
                                               const std::shared_ptr<const transform::WorldModel>& world_model_ptr,
                                               const time::TimePoint time) {
    /* NEWLINE */

    if (world_model_ptr == nullptr) return std::nullopt;

    // Get Position of robots
    const auto passer_affine =
        passing_robot.getCurrentPosition(world_model_ptr, world_model_ptr->getGlobalFrame(), time);
    if (!passer_affine.has_value()) return std::nullopt;

    const auto receiver_affine =
        receiving_robot.getCurrentPosition(world_model_ptr, world_model_ptr->getGlobalFrame(), time);
    if (!receiver_affine.has_value()) return std::nullopt;

    // extract positions to vector
    const Eigen::Vector2d passing_robot_pos = passer_affine->translation();
    const Eigen::Vector2d receiving_robot_pos = receiver_affine->translation();

    // @todo calculate dynamically
    const auto& cs = config_provider::ConfigProvider::getConfigStore();
    const double max_enemy_vel = cs.observer_config.pass_prob_max_enemy_vel;
    const double rotation_factor = cs.observer_config.pass_prob_rotation_factor;

    // calculate the time the passer needs to rotate around the ball
    const auto ball_pos = transform::helper::getBallPosition(*world_model_ptr);
    if (!ball_pos.has_value()) return std::nullopt;
    const auto passer_ball_vec = *ball_pos - passing_robot_pos;
    const auto ball_receiver_vec = receiving_robot_pos - *ball_pos;
    const double passer_angle_delta = utility::calculateAngle(passer_ball_vec, ball_receiver_vec);
    const double rotation_time = passer_angle_delta * rotation_factor;

    const auto passer_receiver_vec = receiving_robot_pos - passing_robot_pos;

    constexpr double BALL_VELOCITY = 6.0;

    /* -------------------- Calculate probability -------------------- */

    double pass_probability = 1.0;

    for (const auto& enemy : world_model_ptr->getVisibleRobots(time)) {
        if (enemy.getTeam() == team) continue;

        const auto enemy_pos = transform::helper::getPosition(transform::RobotHandle(enemy, world_model_ptr));
        if (!enemy_pos.has_value()) continue;

        // get the point on the passer_receiver vector closest to the interceptor
        const Eigen::Vector2d passer_receiver_vec_norm = passer_receiver_vec / passer_receiver_vec.norm();

        const Eigen::Vector2d passer_enemy_diff = passing_robot_pos - *enemy_pos;

        const double dot1 = passer_enemy_diff.dot(passer_receiver_vec_norm);
        const double dot2 = passer_receiver_vec_norm.dot(passer_receiver_vec_norm);
        const Eigen::Vector2d r1 = (dot1 / dot2) * passer_receiver_vec_norm;

        const Eigen::Vector2d p = passing_robot_pos + r1;

        // const Eigen::Vector2d p = Eigen::Vector2d{0, 0};

        // const Eigen::Vector2d p = passing_robot_pos - (passer_receiver_vec_norm * (passing_robot_pos - *enemy_pos) *
        //                                                passer_receiver_vec_norm);

        const double ball_intercept_distance = utility::calculateDistance(p, *ball_pos);
        const double ball_travel_time = ball_intercept_distance / BALL_VELOCITY;

        const double interceptor_coverage_radius = (ball_travel_time + rotation_time) * max_enemy_vel;

        /*
        1. Calculate Points on line from passer to receiver which are exactly ROBOT_COVERAGE_RADIUS far
        away from the enemy
        2. Take the distance of these two points
        3. prob = 2*COVERAGE_RADIUS - calculated_distance
        */

        const auto points =
            calculateCirclePoints(passing_robot_pos, receiving_robot_pos, *enemy_pos, interceptor_coverage_radius);

        if (!points.has_value()) continue;
        const auto [p1, p2] = *points;

        const double dist = utility::calculateDistance(p1, p2);

        const double intercept_probability = 2 * interceptor_coverage_radius - dist;
        pass_probability *= intercept_probability;
    }

    return pass_probability;
}

std::optional<std::pair<Eigen::Vector2d, Eigen::Vector2d>> calculateCirclePoints(const Eigen::Vector2d& robot1,
                                                                                 const Eigen::Vector2d& robot2,
                                                                                 const Eigen::Vector2d& interceptor,
                                                                                 const double interceptor_radius) {
    /*

    The circle-points are the two points which lay on the circle around the interceptor.
    These points describe how likely the enemy is to intercept that pass.

    It works like this:
        1:
            We calculate the line between the passer and the receiver
            We calculate the distance from this line to the interceptor
            if this distance is greater than the radius the interceptor
                covers we the interceptor cant intercept the ball

        2:
            Otherwise we calculate the two previously described points and return them

    */

    // 1: check if interception points exist

    const Eigen::Vector2d passer_interceptor_vec = interceptor - robot1;
    const Eigen::Vector2d passer_receiver_vec = robot2 - robot1;

    const Eigen::Vector3d passer_interceptor_vec3 =
        Eigen::Vector3d{passer_interceptor_vec.x(), passer_interceptor_vec.y(), 0};
    const Eigen::Vector3d passer_receiver_vec3 = Eigen::Vector3d{passer_receiver_vec.x(), passer_receiver_vec.y(), 0};

    const Eigen::Vector3d passer_interceptor_recv_cross = passer_interceptor_vec3.cross(passer_receiver_vec3);
    const double dist_denom = passer_interceptor_recv_cross.norm();
    const double dist_nomin = passer_receiver_vec.norm();
    const double distance = dist_denom / dist_nomin;

    // check that the interceptor is close enough to the line between the passer & receiver
    if (distance < interceptor_radius) return std::nullopt;

    const auto receiver_interceptor_vec = interceptor - robot2;

    const double passer_border = passer_receiver_vec.dot(passer_interceptor_vec);
    const double receiver_border = passer_receiver_vec.dot(receiver_interceptor_vec);

    // check if the interceptor is between the passer and receiver
    if (passer_border < 0 || receiver_border > 0) return std::nullopt;

    // 2: calculate points

    const double rad = interceptor_radius;

    const double x1 = robot1.x();
    const double y1 = robot1.y();

    const double x2 = robot2.x();
    const double y2 = robot2.y();

    const double x3 = interceptor.x();
    const double y3 = interceptor.y();

    const double d = x1 - x3;
    const double e = x2 - x1;

    const double f = y1 - y3;
    const double h = y2 - y1;

    const double t1 = (2.0 * d * e + 2.0 * f * h) / (2.0 * (e * e + h * h));
    const double t2 = t1 * t1;
    const double t3 = (d * d + f * f - rad * rad) / (e * e + h * h);

    const double sqrt_cont = t2 - t3;
    if (sqrt_cont < 0) return std::nullopt;

    const double sqrt_res = sqrt(sqrt_cont);
    const double k1 = -t1 + sqrt_res;
    const double k2 = -t1 - sqrt_res;

    const Eigen::Vector2d p1 = {
        x1 + k1 * (x2 - x1),
        y1 + k1 * (y2 - y1),
    };

    const Eigen::Vector2d p2 = {
        x1 + k2 * (x2 - x1),
        y1 + k2 * (y2 - y1),
    };

    return std::pair{p1, p2};
}

}  // namespace luhsoccer::observer::calculation::pass_probability