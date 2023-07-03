
#include "local_planner_components/features/turn_around_ball_feature.hpp"
#include <Eigen/src/Core/Matrix.h>

#include "local_planner/skills/abstract_shape.hpp"
#include "logger/logger.hpp"
#include "transform/position.hpp"
namespace luhsoccer::local_planner {

Eigen::Vector2d TurnAroundBallFeature::calcArtificialDesiredVelocity(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
    const time::TimePoint& time, const ComponentPosition& observe_position) const {
    auto vec_and_vel_to_ball = this->shape->getTransformToClosestPoint(wm, td, robot, time, observe_position);

    auto align_position_vec = this->align_position.positionObject(wm, td).getCurrentPosition(
        wm, observe_position.positionObject(wm, td), time);
    auto ball_vec = transform::Position("ball").getCurrentPosition(wm, observe_position.positionObject(wm, td), time);
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
            -this->weight.val(wm, td) * localPlannerConfig().feature_turn_around_ball_k.val() * angle * vec_to_ball;

        // calculate distance to ball using vec_and_vel_to_ball
        double distance_to_ball = vec_and_vel_to_ball.vec->norm();
        // LOG_INFO(logger::Logger("TurnAroundBallFeature"), "Distance to Ball: {}", distance_to_ball.val(wm, td));

        auto ball_info = wm->getBallInfo();
        if (ball_info.has_value() && ball_info->robot.has_value()) {
            if (ball_info->state == transform::BallState::IN_ROBOT && ball_info->robot == td.robot) {
                return {0.0, 0.0};
            }
        }

        // check distance to ball using cookies to correct position
        auto start = this->getCookie<double>(td, "start_distance_to_ball");
        if (start.has_value()) {
            desired_velocity = desired_velocity + localPlannerConfig().feature_turn_around_ball_g.val() /
                                                      localPlannerConfig().feature_target_k_v *
                                                      vec_and_vel_to_ball.vec->normalized() *
                                                      (distance_to_ball - start.value());
        } else {
            start = distance_to_ball;
            this->setCookie(td, "start_distance_to_ball", start.value());
        }

        auto robot_vel = wm->getVelocity(robot.getFrame(), observe_position.positionObject(wm, td).getFrame(),
                                         observe_position.positionObject(wm, td).getFrame(), time);
        if (vec_and_vel_to_ball.velocity.has_value() && robot_vel.has_value()) {
            Eigen::Vector2d target_vel = vec_and_vel_to_ball.velocity.value() + robot_vel.value().velocity.head(2);
            desired_velocity += this->weight.val(wm, td) * target_vel;
        }

        // check if distance from ball to robot changed during TurnAroundBall
    }
    return desired_velocity;
}

bool TurnAroundBallFeature::isReached(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                                      const RobotIdentifier& robot, const time::TimePoint& time) const {
    auto vec_and_vel_to_ball = this->shape->getTransformToClosestPoint(wm, td, robot, time);

    auto align_position_vec = this->align_position.positionObject(wm, td).getCurrentPosition(wm, "", time);
    auto ball_vec = transform::Position("ball").getCurrentPosition(wm, "", time);

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
        return std::abs(angle) < this->rotational_tolerance.val(wm, td);
    }
    return false;
}

}  // namespace luhsoccer::local_planner