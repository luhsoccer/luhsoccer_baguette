#include <cmath>
#include <utility>

#include "utils/utils.hpp"

#include "transform_helper/world_model_helper.hpp"
#include "observer/static_observer.hpp"
#include "game_data_provider/game_data_provider.hpp"

#include "config_provider/config_store_main.hpp"
#include "config/observer_config.hpp"

#include "observer/goal_probability/goal_probability.hpp"
#include "observer/pass_probability/pass_probability.hpp"
#include "observer/utility.hpp"
#include "observer/misc/misc.hpp"
#include "observer/static_observer_position.hpp"
#include "observer/continuous_observer.hpp"

namespace luhsoccer::observer::calculation {

namespace {
std::optional<BallHolder> ballPosessionPenaltyArea(const RobotIdentifier goalie,
                                                   const std::vector<RobotIdentifier>& robots,
                                                   const Eigen::Vector2d& ball_pos,
                                                   const std::shared_ptr<const transform::WorldModel> world_model_ptr) {
    if (std::find(robots.begin(), robots.end(), goalie) != robots.end()) {
        return BallHolder(transform::RobotHandle(goalie, world_model_ptr), std::nullopt, true);
    }

    // find closest ally to ball
    std::optional<RobotIdentifier> closest_ally{std::nullopt};
    double closest_distance = std::numeric_limits<double>::max();

    for (const auto& ally : robots) {
        const auto ally_pos = transform::helper::getPosition(transform::RobotHandle(ally, world_model_ptr));
        if (ally_pos.has_value()) {
            double dist = utility::calculateDistance(*ally_pos, ball_pos);
            if (dist < closest_distance) {
                dist = closest_distance;
                closest_ally = ally;
            }
        }
    }

    if (closest_ally.has_value()) {
        return BallHolder(transform::RobotHandle(*closest_ally, world_model_ptr), std::nullopt, true);
    }

    return std::nullopt;
}
}  // namespace

double calculateGoalProbability(const transform::RobotHandle& handle, const time::TimePoint time) {
    return calculateGoalProbability(handle.getPosition(), handle.getTeam(), handle.getWorldModel().lock(), time);
}

std::optional<BallHolder> calculateBallPosession(const game_data_provider::GameDataProvider& gdp) {
    // NEWLINE

    const auto& observer_config = config_provider::ConfigProvider::getConfigStore().observer_config;
    const double min_required_distance = observer_config.ball_posession_threshhold;
    constexpr double DRIBBLER_MID_DIST = 0.075;

    const auto& observer = *gdp.getObserver();
    const auto old_ball_carrier = observer.getBallCarrier();

    const std::shared_ptr<const transform::WorldModel> world_model_ptr = gdp.getWorldModel();

    // Get Position of the Ball
    const auto ball_pos = transform::helper::getBallPosition(*world_model_ptr);
    if (!ball_pos.has_value()) return std::nullopt;

    // Check if the Ball-Filter says that a robot has the ball and if so return that robot
    // If this is the case we also need to check if the robot has moved more than 1m away from where it has obtained the
    // ball to see if it is still allowed to move
    const auto ball_info = world_model_ptr->getBallInfo();
    if (ball_info.has_value()) {
        if (ball_info->state == transform::BallState::IN_ROBOT && ball_info->robot.has_value()) {
            auto ball_holder = BallHolder(transform::RobotHandle(*ball_info->robot, world_model_ptr), ball_pos, true);

            // Get the position where the ball was last obtained. If this is not set (eg. std::nullopt) the ball was
            // previously no in any team's poession so movement is allowed
            const auto ball_obtained_pos = world_model_ptr->getLastBallObtainPosition();
            if (!ball_obtained_pos.has_value()) {
                return ball_holder;
            }

            ball_holder.ball_obtained_pos = ball_obtained_pos->second.translation();

            // get the distance from the balls current position to the position where the ball was obtained
            const double distance_from_ball_obtained_pos = (*ball_pos - ball_obtained_pos->second.translation()).norm();

            // If that distance is greater than 1m no movement is allowed
            const double max_ball_movement_diatance =
                config_provider::ConfigProvider::getConfigStore().observer_config.max_ball_movement_distance;

            if (distance_from_ball_obtained_pos > max_ball_movement_diatance) {
                ball_holder.movement_allowed = false;
            }

            return ball_holder;
        }
    }

    // check if the ball can be in any team's penalty area
    const double penaly_area_width = world_model_ptr->getFieldData().penalty_area_width / 2;
    if (ball_pos->y() < penaly_area_width && ball_pos->y() > -penaly_area_width) {
        // ALLY Penalty area
        const double penalty_area_begin =
            world_model_ptr->getFieldData().size.x() + world_model_ptr->getFieldData().penalty_area_depth;
        if (ball_pos->x() < -penalty_area_begin) {
            RobotIdentifier ally_goalie = gdp.getGoalie();

            const auto& ally_robots = world_model_ptr->getVisibleRobots<Team::ALLY>();

            return ballPosessionPenaltyArea(ally_goalie, ally_robots, *ball_pos, world_model_ptr);
        }

        // ENEMY penalty area
        if (ball_pos->x() > penalty_area_begin) {
            RobotIdentifier enemy_goalie = gdp.getEnemyGoalie();

            const auto& enemy_robots = world_model_ptr->getVisibleRobots<Team::ENEMY>();

            return ballPosessionPenaltyArea(enemy_goalie, enemy_robots, *ball_pos, world_model_ptr);
        }
    }

    // Determine closest robot to ball
    std::optional<BallHolder> ball_holder = std::nullopt;
    double min_distance_to_ball = std::numeric_limits<double>::max();

    for (const auto& identifier : world_model_ptr->getVisibleRobots()) {
        // Get Positional(&rotational) infos about the current Robot
        const auto pos = transform::helper::getPositionAndRotation(transform::RobotHandle(identifier, world_model_ptr));
        if (!pos.has_value()) continue;

        // Vector from robot center to middle point of dribbler
        Eigen::Vector2d direction_dribbler = utility::calculateRotationVector(pos->z());

        // Calculate the mid point of the dribbler
        Eigen::Vector2d dribbler_mid = pos->head<2>() + direction_dribbler * DRIBBLER_MID_DIST;

        // Vector from the dribbler-mid to the ball
        Eigen::Vector2d dribbler_mid_to_ball = dribbler_mid - *ball_pos;

        // dot product of vector (Dribbler->Ball) and (Dribbler direction)
        // Results in large values if the dribbler direction aligns with the ball-vector; 0 if it is orthogonal (to the
        // right or left); negative value if ball is behind robot
        double factor = dribbler_mid_to_ball.dot(direction_dribbler);

        // this factor is needed for the calculation of a orthogonal projection
        Eigen::Vector2d orthogonal_projection = direction_dribbler * factor;

        double distance_orthogonal_projection = orthogonal_projection.norm();

        // if the orthoganal projection is larger then the dribbler, the robot will not be considered in the following
        // calculations
        if (distance_orthogonal_projection > DRIBBLER_MID_DIST / 2 + min_required_distance) {
            continue;
        }

        // Get the distance to the ball from the tip of the 'orthogonal_projection' vector (including the dribbler pos)
        const double distance_ball = (orthogonal_projection + dribbler_mid - *ball_pos).norm();

        // the additionsal distance the robot which currently 'has' the ball has as its ball-having-radius
        // The area in which the ball carrier holds the ball should be 35% bigger than the area when he gets the ball
        double distance_factor = (old_ball_carrier.has_value() && identifier == old_ball_carrier->getID()) ? 1.35 : 1.0;

        // check if the ball is inside the required min distance & if the current robot is closest to ball
        if (distance_ball < min_required_distance * distance_factor && distance_ball < min_distance_to_ball) {
            // Because in this case the "ball holder" only controlls the rough area of the Ball and it doesnt really
            // have the ball in its dribbler, the movement is allowed AND no ball obtained position is set
            ball_holder = BallHolder(transform::RobotHandle(identifier, world_model_ptr), std::nullopt, true);
            min_distance_to_ball = distance_ball;
        }
    }

    // if no ball holder was found return nullopt
    if (!ball_holder.has_value()) return std::nullopt;

    return ball_holder;
}

double calculateThreatLevel(const transform::RobotHandle& enemy_handle) {
    return calculateThreatLevel(enemy_handle.getPosition(), enemy_handle.getWorldModel().lock(), time::TimePoint(0));
}

std::optional<bool> isPassLineDefended(const transform::RobotHandle& passer, const transform::RobotHandle& receiver,
                                       const time::TimePoint time) {
    return isPassLineDefended(passer.getPosition(), receiver.getPosition(), passer.getTeam(),
                              passer.getWorldModel().lock(), time);
}

std::optional<bool> isOutgoingPassDefended(const transform::RobotHandle& ball_holder, const time::TimePoint time) {
    return isOutgoingPassDefended(ball_holder.getPosition(), ball_holder.getTeam(), ball_holder.getWorldModel().lock(),
                                  ball_holder.getID(), time);
}

std::optional<Eigen::Vector2d> calculateFutureBallPos(std::shared_ptr<const transform::WorldModel> world_model_ptr,
                                                      time::Duration time_from_now) {
    // Define Constants
    const double carpet_drag_coeff =
        config_provider::ConfigProvider::getConfigStore().observer_config.future_ball_pos_drag_coeff;

    // get the position of the ball
    auto ball_pos = transform::helper::getBallPosition(*world_model_ptr);
    if (!ball_pos.has_value()) return std::nullopt;

    // get the velocity of the Ball
    const auto ball_vel = transform::helper::getBallVelocity(*world_model_ptr);
    if (!ball_vel.has_value()) return std::nullopt;

    // get the length of the velocity vector
    const double ball_vel_abs = ball_vel->norm();

    // get the x and y factor of the moved distance
    const Eigen::Vector2d factors = {ball_vel->x() / ball_vel_abs, ball_vel->y() / ball_vel_abs};

    // PROBABLY WRONG FORMULA
    // The distance the ball traveled after *time_from_now* while under the influence of rolling-resistance
    // Formula: distance = (initial velocity * time) - (friction_coef * time^2) / 2
    double moved_distance = (ball_vel_abs * time_from_now.asSec()) -
                            (carpet_drag_coeff * time_from_now.asSec() * time_from_now.asSec()) / 2.0;

    moved_distance = std::max(0.0, moved_distance);

    // Calculate the future position of the ball
    *ball_pos = *ball_pos + factors * moved_distance;

    return ball_pos;
}

std::optional<Eigen::Vector2d> calculateBestGoalPoint(const transform::RobotHandle& robot,
                                                      std::optional<RobotIdentifier> id_to_ignore,
                                                      const time::TimePoint time) {
    return calculateBestGoalPoint(robot.getPosition(), robot.getTeam(), robot.getWorldModel().lock(), id_to_ignore,
                                  time);
}

std::optional<Eigen::Vector2d> calculateMostLikelyShootPoint(const transform::RobotHandle& enemy,
                                                             const game_data_provider::GameDataProvider& gdp,
                                                             const time::TimePoint time) {
    return calculateBestGoalPoint(enemy.getPosition(), enemy.getTeam(), enemy.getWorldModel().lock(), gdp.getGoalie(),
                                  time);
}

std::optional<std::pair<Eigen::Vector2d, bool>> calculateShootPoint(const transform::RobotHandle& robot,
                                                                    const time::TimePoint time) {
    return calculateShootPoint(robot.getPosition(), robot.getTeam(), robot.getWorldModel().lock(), time);
}

std::optional<AllyRobot::BestPassReceiver> calculateBestPassReceiver(const transform::RobotHandle& passing_robot_handle,
                                                                     std::vector<RobotIdentifier> allies_to_evaluate,
                                                                     const time::TimePoint time) {
    return calculateBestPassReceiver(passing_robot_handle.getPosition(), passing_robot_handle.getTeam(),
                                     passing_robot_handle.getWorldModel().lock(), std::move(allies_to_evaluate),
                                     passing_robot_handle.getID(), time);
}

double calculatePassProbability(const transform::RobotHandle& passing_robot, const RobotIdentifier& receiving_robot,
                                const std::shared_ptr<const transform::WorldModel>& world_model_ptr,
                                const time::TimePoint time) {
    auto pass_prob = pass_probability::calculatePassProbability(passing_robot, receiving_robot, world_model_ptr, time);
    if (pass_prob.has_value()) {
        return pass_prob.value();
    }
    return 0;
}

double calculateInterceptionScore(const transform::RobotHandle& handle, const time::TimePoint time) {
    const auto world_model = handle.getWorldModel().lock();
    if (world_model == nullptr) return 0;

    const auto ball_info = world_model->getBallInfo();
    if (ball_info.has_value() && ball_info->state == transform::BallState::IN_ROBOT && ball_info->robot.has_value()) {
        if (handle.getID() == *ball_info->robot) return 0;
    }

    const auto pos = transform::helper::getPosition(handle, time);
    if (!pos.has_value()) return 0;

    const auto ball_pos = transform::helper::getBallPosition(*world_model);
    if (!ball_pos.has_value()) return 0;

    const auto ball_vel_vec = transform::helper::getBallVelocity(*world_model, time);
    if (!ball_vel_vec.has_value()) return 0;

    const auto& cs = config_provider::ConfigProvider::getConfigStore();

    const double min_ball_vel_threshold = cs.observer_config.interceptor_min_ball_vel_threshold;

    if (ball_vel_vec->norm() < min_ball_vel_threshold) return 0.0;

    const auto ball_robot_vec = *pos - *ball_pos;

    const double max_dist_to_ball_in_sec = cs.observer_config.interceptor_max_distance_in_ball_vel;
    const double max_dist_to_ball = ball_vel_vec->norm() * max_dist_to_ball_in_sec;

    const double distance_robot_to_ball = ball_robot_vec.norm();

    if (distance_robot_to_ball > max_dist_to_ball) return 0.0;

    const double max_robot_vel = cs.observer_config.interceptor_max_robot_vel;
    const double alpha_factor = cs.observer_config.interceptor_alpha_factor;

    const auto [is_viable, angle] =
        misc::isInsideAngle(ball_robot_vec, ball_vel_vec->head<2>(), max_robot_vel, alpha_factor);

    if (!is_viable) return 0.0;

    return 10.0 / angle;
}

std::optional<bool> calculateInterceptionRobotIsViable(const transform::RobotHandle& handle,
                                                       const time::TimePoint time) {
    const auto world_model = handle.getWorldModel().lock();
    if (world_model == nullptr) return std::nullopt;

    const auto ball_info = world_model->getBallInfo();
    if (ball_info.has_value() && ball_info->state == transform::BallState::IN_ROBOT && ball_info->robot.has_value()) {
        if (handle.getID() == *ball_info->robot) return false;
    }

    const auto pos = transform::helper::getPosition(handle, time);
    if (!pos.has_value()) return std::nullopt;

    const auto ball_pos = transform::helper::getBallPosition(*world_model);
    if (!ball_pos.has_value()) return std::nullopt;

    const auto ball_vel_vec = transform::helper::getBallVelocity(*world_model, time);
    if (!ball_vel_vec.has_value()) return std::nullopt;

    const auto& cs = config_provider::ConfigProvider::getConfigStore();

    const auto ball_robot_vec = *pos - *ball_pos;

    const double min_ball_vel_threshold = cs.observer_config.interceptor_min_ball_vel_threshold;
    const double ball_vel_abs = ball_vel_vec->norm();
    if (ball_vel_abs < min_ball_vel_threshold) return std::nullopt;

    const double max_robot_vel = cs.observer_config.interceptor_max_robot_vel;
    const double alpha_factor = cs.observer_config.interceptor_alpha_factor;

    return std::get<0>(misc::isInsideAngle(ball_robot_vec, ball_vel_vec->head<2>(), max_robot_vel, alpha_factor));
}

std::optional<transform::RobotHandle> calculateBestInterceptor(
    const std::shared_ptr<const transform::WorldModel>& world_model, std::vector<RobotIdentifier> viable_interceptors,
    const time::TimePoint time) {
    const auto ball_info = world_model->getBallInfo();

    if (ball_info.has_value() && ball_info->state == transform::BallState::IN_ROBOT && ball_info->robot.has_value() &&
        ball_info->robot->isEnemy()) {
        return std::nullopt;
    }

    const auto ball_vel_vec = transform::helper::getBallVelocity(*world_model, time);
    if (!ball_vel_vec.has_value()) return std::nullopt;

    const auto ball_pos = transform::helper::getBallPosition(*world_model);
    if (!ball_pos.has_value()) return std::nullopt;

    const auto& cs = config_provider::ConfigProvider::getConfigStore();

    const double alpha_factor = cs.observer_config.interceptor_alpha_factor;
    const double max_robot_vel = cs.observer_config.interceptor_max_robot_vel;
    const double min_ball_vel_threshold = cs.observer_config.interceptor_min_ball_vel_threshold;
    const double max_dist_to_ball_in_sec = cs.observer_config.interceptor_max_distance_in_ball_vel;

    const double ball_vel_abs = ball_vel_vec->norm();
    if (ball_vel_abs < min_ball_vel_threshold) return std::nullopt;

    const double max_dist_to_ball = ball_vel_abs * max_dist_to_ball_in_sec;

    std::optional<RobotIdentifier> best_id_in_angle = std::nullopt;
    double best_id_ball_angle = std::numeric_limits<double>::max();

    const bool ball_in_robot =
        ball_info.has_value() && ball_info->state == transform::BallState::IN_ROBOT && ball_info->robot.has_value();

    if (viable_interceptors.empty()) {
        viable_interceptors = world_model->getVisibleRobots<Team::ALLY>(time);
    }

    for (const auto& id : viable_interceptors) {
        if (ball_in_robot && id == ball_info->robot) continue;

        const auto pos_affine = world_model->getTransform(id.getFrame());
        if (!pos_affine.has_value()) return std::nullopt;
        const auto pos = pos_affine->transform.translation();

        const auto ball_robot_vec = pos - *ball_pos;
        // const bool robot_viable =
        //     misc::isInsideAngle(ball_robot_vec, ball_vel_vec->head<2>(), max_robot_vel, alpha_factor);

        // if (!robot_viable) continue;

        // check whether the robot is viable
        const double alpha = (std::atan(max_robot_vel / ball_vel_abs) * 180.0 / L_PI) * alpha_factor;
        const double robot_ball_angle = utility::calculateAngle(ball_robot_vec, ball_vel_vec->head<2>());
        const double robot_ball_angle_cropped = abs(cropAngle(robot_ball_angle));
        const double robot_ball_angle_deg = robot_ball_angle_cropped * 180.0 / L_PI;
        if (robot_ball_angle_deg > alpha) continue;

        const double distance_robot_to_ball = ball_robot_vec.norm();

        if (robot_ball_angle_deg < best_id_ball_angle && distance_robot_to_ball < max_dist_to_ball) {
            best_id_ball_angle = robot_ball_angle_deg;
            best_id_in_angle = id;
        }
    }

    if (!best_id_in_angle.has_value()) return std::nullopt;

    return transform::RobotHandle(*best_id_in_angle, world_model);
}

}  // namespace luhsoccer::observer::calculation
