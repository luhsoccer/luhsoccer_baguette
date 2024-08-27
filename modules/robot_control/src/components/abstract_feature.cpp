

#include "robot_control/components/abstract_feature.hpp"

#include "robot_control/components/component_data.hpp"
#include "config/robot_control_visualization_config.hpp"
#include "marker_adapter.hpp"
#include "robot_control/components/abstract_shape.hpp"
#include "config/robot_control_config.hpp"
#include "robot_control/components/component_util.hpp"

namespace luhsoccer::robot_control {

AbstractFeature::AbstractFeature(std::shared_ptr<const AbstractShape> shape, DoubleComponentParam weight)
    : weight(std::move(weight)), shape(std::move(shape)){};

double AbstractFeature::getWeight(const ComponentData& comp_data) const { return this->weight.val(comp_data); };

void AbstractFeature::visualizeAdditionalOptions(const ComponentData& comp_data) const {
    auto color = this->getVisualizationColor(comp_data);
    if (robotControlVisualizationConfig().display_shape_vectors) {
        auto to_closest_point = this->shape->getTransformToClosestPoint(comp_data, {comp_data.robot.getFrame()},
                                                                        {comp_data.robot.getFrame()});
        if (to_closest_point.vec.has_value()) {
            marker::Line shape_vec(comp_data.robot.getFrame());
            shape_vec.setLinePoints({0.0, 0.0}, {to_closest_point.vec->x(), to_closest_point.vec->y()});
            shape_vec.setColor(color);
            shape_vec.setFrameLocked(true);
            comp_data.ma.displayMarker(shape_vec);
        }
    }
    if (robotControlVisualizationConfig().display_shape_vectors_to_center) {
        auto to_center_vec = this->shape->getCenter(comp_data);
        if (to_center_vec.has_value()) {
            marker::Line shape_vec(comp_data.robot.getFrame());
            shape_vec.setLinePoints({0.0, 0.0}, {to_center_vec->x(), to_center_vec->y()});
            shape_vec.setColor(color);
            shape_vec.setFrameLocked(true);
            comp_data.ma.displayMarker(shape_vec);
        }
    }
}

[[nodiscard]] VectorWithVelocityStamped AbstractFeature::getTransformToClosestPointFromShape(
    const ComponentData& comp_data, const transform::Position& robot_position,
    const transform::Position& observe_position) const {
    std::unique_ptr<MarkerAdapter> ma = std::make_unique<MarkerAdapter>();
    if (robotControlVisualizationConfig().display_shapes.val()) {
        ma = std::make_unique<MarkerAdapter>(comp_data.ma);
        ma->setOverrideColor(this->getVisualizationColor(comp_data));

        if (robotControlVisualizationConfig().display_shape_influence) {
            auto influence = this->getCookie<bool>(comp_data.td, "influence");
            if (influence.has_value() && !influence.value()) {
                ma->setOverrideOccupancy(robotControlVisualizationConfig().shape_influence_value);
            }
        }
    }

    ComponentData shape_comp_data(comp_data, *ma);

    auto robot_to_feature = this->shape->getTransformToClosestPoint(shape_comp_data, robot_position, observe_position);
    this->visualizeAdditionalOptions(shape_comp_data);
    return robot_to_feature;
}

AbstractTargetFeature::AbstractTargetFeature(std::shared_ptr<const AbstractShape> shape, bool reachable,
                                             const DoubleComponentParam& weight)
    : AbstractFeature(std::move(shape), weight), reachable(reachable){};

std::optional<std::pair<Eigen::Vector2d, Eigen::Vector2d>> AbstractObstacle::getVecAndVelocity(
    const ComponentData& comp_data, const transform::Position& robot_position,
    const transform::Position& observe_position) const {
    std::pair<Eigen::Vector2d, Eigen::Vector2d> vec_and_vel;
    auto robot_to_feature = this->getTransformToClosestPointFromShape(comp_data, robot_position, observe_position);
    if (robot_to_feature.vec.has_value() && robot_to_feature.vec->norm() != 0.0) {
        vec_and_vel.first = robot_to_feature.vec.value();
    } else {
        return std::nullopt;
    }
    if (robot_to_feature.velocity.has_value()) {
        vec_and_vel.second = -robot_to_feature.velocity.value();
    } else {
        return std::nullopt;
    }
    return vec_and_vel;
}

std::optional<Eigen::Vector2d> AbstractObstacle::getVelocityCommand(const ComponentData& comp_data,
                                                                    const Eigen::Vector2d& command_velocity,
                                                                    bool bypass_direction, double mean_target_distance,
                                                                    const transform::Position& robot_position,
                                                                    const transform::Position& observe_position) const {
    // check weight
    this->setCookie(comp_data.td, "influence", false);

    constexpr double SMALL_VALUE = 0.0001;
    if (this->weight.val(comp_data) < SMALL_VALUE) return std::nullopt;

    auto vec_and_vel = this->getVecAndVelocity(comp_data, robot_position, observe_position);
    if (!vec_and_vel.has_value()) return std::nullopt;

    Eigen::Vector2d vec_to_obstacle = vec_and_vel->first;
    Eigen::Vector2d vel_to_obstacle = vec_and_vel->second;

    // check influence distance and mean weighted target
    if (vec_to_obstacle.norm() > std::min(this->influence_distance.val(comp_data), mean_target_distance))
        return std::nullopt;

    // check moving away from obstacle
    auto robot_vel = robot_position.getVelocity(comp_data.wm, observe_position, observe_position, comp_data.time);
    if (!robot_vel.has_value()) return std::nullopt;

    if (robot_vel->head<2>().dot(vec_to_obstacle) < 0.0) return std::nullopt;

    Eigen::Vector3d vec_to_obstacle_3d = {vec_to_obstacle.x(), vec_to_obstacle.y(), 0.0};
    Eigen::Vector3d vel_to_obstacle_3d = {vel_to_obstacle.x(), vel_to_obstacle.y(), 0.0};
    Eigen::Vector3d command_velocity_3d = {command_velocity.x(), command_velocity.y(), 0.0};

    Eigen::Vector3d magnetic_field_vector = {0.0, 0.0, bypass_direction ? 1.0 : -1.0};

    Eigen::Vector3d current_vec = vec_to_obstacle_3d.cross(magnetic_field_vector);

    Eigen::Vector3d triple_product = command_velocity_3d.normalized().cross(current_vec.cross(vel_to_obstacle_3d));

    this->setCookie(comp_data.td, "influence", true);
    return robotControlConfig().obstacle_k_cf / vec_to_obstacle_3d.squaredNorm() * triple_product.head(2);
}

AbstractObstacle::AbstractObstacle(std::shared_ptr<const AbstractShape> shape, const DoubleComponentParam& weight,
                                   const std::optional<DoubleComponentParam>& critical_distance,
                                   const std::optional<DoubleComponentParam>& influence_distance)
    : AbstractFeature(std::move(shape), weight),
      critical_distance(critical_distance.has_value() ? critical_distance.value()
                                                      : robotControlConfig().obstacle_collision_distance),
      influence_distance(influence_distance.has_value() ? influence_distance.value()
                                                        : robotControlConfig().obstacle_influence_distance){};

}  // namespace luhsoccer::robot_control