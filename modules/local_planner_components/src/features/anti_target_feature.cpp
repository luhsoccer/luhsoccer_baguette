

#include "local_planner_components/features/anti_target_feature.hpp"

#include "local_planner/skills/abstract_shape.hpp"
#include "logger/logger.hpp"
namespace luhsoccer::local_planner {

Eigen::Vector2d AntiTargetFeature::calcArtificialDesiredVelocity(const std::shared_ptr<const transform::WorldModel>& wm,
                                                                 const TaskData& td, const RobotIdentifier& robot,
                                                                 const time::TimePoint& time,
                                                                 const ComponentPosition& observe_position) const {
    auto vec_and_vel_to_shape = this->shape->getTransformToClosestPoint(wm, td, robot, time, observe_position);

    Eigen::Vector2d desired_velocity = Eigen::Vector2d::Zero();
    if (vec_and_vel_to_shape.vec.has_value()) {
        if (vec_and_vel_to_shape.vec->norm() < influence_distance.val(wm, td) &&
            vec_and_vel_to_shape.vec->norm() != 0.0) {
            desired_velocity = -(1 / vec_and_vel_to_shape.vec->squaredNorm()) * *vec_and_vel_to_shape.vec *
                               this->weight.val(wm, td) * this->feature_k.val(wm, td);
        }
    }
    return desired_velocity;
};

}  // namespace luhsoccer::local_planner