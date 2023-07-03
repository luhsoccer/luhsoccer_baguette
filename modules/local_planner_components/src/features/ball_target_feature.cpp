

#include "local_planner_components/features/ball_target_feature.hpp"

#include "local_planner/skills/abstract_shape.hpp"
#include "logger/logger.hpp"
namespace luhsoccer::local_planner {

Eigen::Vector2d BallTargetFeature::calcArtificialDesiredVelocity(const std::shared_ptr<const transform::WorldModel>& wm,
                                                                 const TaskData& td, const RobotIdentifier& robot,
                                                                 const time::TimePoint& time,
                                                                 const ComponentPosition& observe_position) const {
    auto vec_and_vel_to_shape = this->shape->getTransformToClosestPoint(wm, td, robot, time, observe_position);

    Eigen::Vector2d desired_velocity = Eigen::Vector2d::Zero();
    if (vec_and_vel_to_shape.vec) {
        desired_velocity =
            this->weight.val(wm, td) * this->k_g.val(wm, td) / this->k_v.val(wm, td) * *vec_and_vel_to_shape.vec;
    }

    auto robot_vel = wm->getVelocity(robot.getFrame(), observe_position.positionObject(wm, td).getFrame(),
                                     observe_position.positionObject(wm, td).getFrame(), time);
    if (!ignore_velocity.val(wm, td) && vec_and_vel_to_shape.velocity.has_value() && robot_vel.has_value()) {
        Eigen::Vector2d target_vel = vec_and_vel_to_shape.velocity.value() + robot_vel.value().velocity.head(2);
        desired_velocity += this->weight.val(wm, td) * target_vel;
    }

    return desired_velocity;
};

bool BallTargetFeature::isReached(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/,
                                  const RobotIdentifier& robot, const time::TimePoint& time) const {
    auto ball_info = wm->getBallInfo(time);
    return ball_info.has_value() && ball_info->robot.has_value() && ball_info->robot.value() == robot;
};

}  // namespace luhsoccer::local_planner