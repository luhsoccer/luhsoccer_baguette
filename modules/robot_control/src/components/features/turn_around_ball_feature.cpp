
#include "robot_control/components/features/turn_around_ball_feature.hpp"
#include "robot_control/components/component_data.hpp"
#include "config/robot_control_config.hpp"

namespace luhsoccer::robot_control {

std::pair<Eigen::Vector2d, Eigen::Vector2d> TurnAroundBallFeature::calcArtificialDesiredVelocity(
    const ComponentData& comp_data, const std::vector<std::shared_ptr<const AbstractShape>>& /*restricted_areas*/,
    const transform::Position& robot_position,

    const transform::Position& observe_position) const {
    auto vec_and_vel_to_ball = this->getTransformToClosestPointFromShape(comp_data, robot_position, observe_position);

    auto align_position_vec = this->align_position.positionObject(comp_data).getCurrentPosition(
        comp_data.wm, observe_position, comp_data.time);
    auto ball_vec = transform::Position("ball").getCurrentPosition(comp_data.wm, observe_position, comp_data.time);
    Eigen::Vector2d desired_velocity = Eigen::Vector2d::Zero();

    if (vec_and_vel_to_ball.vec && align_position_vec.has_value() && ball_vec.has_value()) {
        Eigen::Vector2d ball_to_align_position = align_position_vec->translation() - ball_vec->translation();

        // calculate angle between vec_and_vel_to_ball and vec_and_vel_to_align_position using atan2
        double angle = std::atan2(ball_to_align_position.y(), ball_to_align_position.x()) -
                       std::atan2(vec_and_vel_to_ball.vec->y(), vec_and_vel_to_ball.vec->x());

        if (angle > L_PI) {
            angle -= 2 * L_PI;
        } else if (angle < -L_PI) {
            angle += 2 * L_PI;
        }

        // calculate orthogonal vector to vec_and_vel_to_ball
        Eigen::Vector2d vec_to_ball =
            Eigen::Vector2d(-vec_and_vel_to_ball.vec->y(), vec_and_vel_to_ball.vec->x()).normalized();

        desired_velocity =
            -this->weight.val(comp_data) * robotControlConfig().turn_around_ball_rot_k_p * angle * vec_to_ball;

        // calculate distance to ball using vec_and_vel_to_ball
        double distance_to_ball = vec_and_vel_to_ball.vec->norm();
        // LOG_INFO(logger::Logger("TurnAroundBallFeature"), "Distance to Ball: {}", distance_to_ball.val(comp_data));

        auto ball_info = comp_data.wm->getBallInfo();
        if (ball_info.has_value() && ball_info->robot.has_value()) {
            if (ball_info->state == transform::BallState::IN_ROBOT && ball_info->robot == comp_data.td.robot) {
                return {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
            }
        }

        // check distance to ball using cookies to correct position
        auto start = this->getCookie<double>(comp_data.td, "start_distance_to_ball");
        if (start.has_value()) {
            desired_velocity = desired_velocity + robotControlConfig().turn_around_ball_trans_k_p.val() *
                                                      vec_and_vel_to_ball.vec->normalized() *
                                                      (distance_to_ball - start.value());
        } else {
            start = distance_to_ball;
            this->setCookie(comp_data.td, "start_distance_to_ball", start.value());
        }

        auto robot_vel = robot_position.getVelocity(comp_data.wm, observe_position.getFrame(),
                                                    observe_position.getFrame(), comp_data.time);
        if (vec_and_vel_to_ball.velocity.has_value() && robot_vel.has_value()) {
            Eigen::Vector2d target_vel = vec_and_vel_to_ball.velocity.value() + robot_vel->head(2);
            desired_velocity += this->weight.val(comp_data) * target_vel;
        }

        // check if distance from ball to robot changed during TurnAroundBall
        return {desired_velocity, vec_and_vel_to_ball.vec.value()};
    }
    return {desired_velocity, Eigen::Vector2d::Zero()};
}

bool TurnAroundBallFeature::isReached(const ComponentData& comp_data) const {
    auto vec_and_vel_to_ball = this->getTransformToClosestPointFromShape(comp_data, comp_data.robot.getFrame(), "");

    auto align_position_vec =
        this->align_position.positionObject(comp_data).getCurrentPosition(comp_data.wm, "", comp_data.time);
    auto ball_vec = transform::Position("ball").getCurrentPosition(comp_data.wm, "", comp_data.time);

    if (vec_and_vel_to_ball.vec && align_position_vec.has_value() && ball_vec.has_value()) {
        Eigen::Vector2d ball_to_align_position = align_position_vec->translation() - ball_vec->translation();
        // calculate angle between vec_and_vel_to_ball and vec_and_vel_to_align_position using atan2
        double angle = std::atan2(ball_to_align_position.y(), ball_to_align_position.x()) -
                       std::atan2(vec_and_vel_to_ball.vec->y(), vec_and_vel_to_ball.vec->x());

        if (angle > L_PI) {
            angle -= 2 * L_PI;
        } else if (angle < -L_PI) {
            angle += 2 * L_PI;
        }
        return std::abs(angle) < this->rotational_tolerance.val(comp_data);
    }
    return false;
}

}  // namespace luhsoccer::robot_control