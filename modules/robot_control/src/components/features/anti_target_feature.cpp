#include "robot_control/components/features/anti_target_feature.hpp"

namespace luhsoccer::robot_control {

std::pair<Eigen::Vector2d, Eigen::Vector2d> AntiTargetFeature::calcArtificialDesiredVelocity(
    const ComponentData& comp_data, const std::vector<std::shared_ptr<const AbstractShape>>& /*restricted_areas*/,
    const transform::Position& robot_position, const transform::Position& observe_position) const {
    auto vec_and_vel_to_shape = this->getTransformToClosestPointFromShape(comp_data, robot_position, observe_position);

    Eigen::Vector2d desired_velocity = Eigen::Vector2d::Zero();
    if (vec_and_vel_to_shape.vec.has_value()) {
        if (vec_and_vel_to_shape.vec->norm() < influence_distance.val(comp_data) &&
            vec_and_vel_to_shape.vec->norm() != 0.0) {
            desired_velocity = -(1 / vec_and_vel_to_shape.vec->squaredNorm()) * *vec_and_vel_to_shape.vec *
                               this->weight.val(comp_data) * this->k_ip.val(comp_data);
        }
    }
    return {desired_velocity, Eigen::Vector2d::Zero()};
};

}  // namespace luhsoccer::robot_control