#include <utility>

#include "robot_control/components/features/obstacle_feature.hpp"
#include "config/robot_control_config.hpp"
#include "robot_control/components/component_data.hpp"
#include "config/robot_control_visualization_config.hpp"
#include "marker_adapter.hpp"
#include "robot_control/components/shapes/circle_shape.hpp"
#include "robot_control/components/shapes/point_shape.hpp"

namespace luhsoccer::robot_control {

RobotObstacleFeature::RobotObstacleFeature(const RobotIdentifier& robot, const DoubleComponentParam& weight,
                                           const std::optional<DoubleComponentParam>& critical_distance,
                                           const std::optional<DoubleComponentParam>& influence_distance,
                                           DoubleComponentParam k_obstacle)
    : ObstacleFeature(
          CircleShape(transform::Position(robot.getFrame()), robotControlConfig().obstacle_robot_radius, true), weight,
          critical_distance, influence_distance, std::move(k_obstacle)),
      robot(robot) {}

std::vector<marker::Point> getArrowPoints(bool right) {
    constexpr double RADIUS = 0.15;
    constexpr int CIRCLE_FACES = 80;
    constexpr double ARC_ANGLE = 270.0 / 180.0 * L_PI;
    constexpr double ARROW_SIZE = 0.05;

    std::vector<marker::Point> p;
    p.reserve(CIRCLE_FACES + 2);

    // arrow head
    double y_offset = ARROW_SIZE;
    if (!right) y_offset *= -1;
    p.emplace_back(RADIUS + ARROW_SIZE, y_offset);
    p.emplace_back(RADIUS, 0);
    p.emplace_back(RADIUS - ARROW_SIZE, y_offset);
    p.emplace_back(RADIUS, 0);

    for (int i = 0; i < CIRCLE_FACES; i++) {
        double angle = ARC_ANGLE * i / CIRCLE_FACES;
        if (!right) angle *= -1;
        p.emplace_back(RADIUS * cos(angle), RADIUS * sin(angle));
    }

    return p;
}

std::optional<Eigen::Vector2d> ObstacleFeature::getVelocityCommand(const ComponentData& comp_data,
                                                                   const Eigen::Vector2d& command_velocity,
                                                                   bool bypass_direction, double mean_target_distance,
                                                                   const transform::Position& robot_position,
                                                                   const transform::Position& observe_position) const {
    std::optional<Eigen::Vector2d> res = AbstractObstacle::getVelocityCommand(
        comp_data, command_velocity, bypass_direction, mean_target_distance, robot_position, observe_position);

    // visualization
    if (robotControlVisualizationConfig().display_bypass_directions) {
        auto center = this->getShape()->getCenter(comp_data, "");
        if (center.has_value()) {
            marker::LineStrip arrow({"", center->x(), center->y()});
            arrow.setPoints(getArrowPoints(!bypass_direction));

            double occupancy = 1.0;
            if (robotControlVisualizationConfig().display_shape_influence) {
                auto influence = this->getCookie<bool>(comp_data.td, "influence");
                if (influence.has_value() && !influence.value()) {
                    occupancy = robotControlVisualizationConfig().shape_influence_value;
                }
            }

            arrow.setColor(bypass_direction ? marker::Color::BLUE(occupancy) : marker::Color::YELLOW(occupancy));
            constexpr double ARROW_HEIGHT = 0.03;  // so that the arrow is above the shape
            arrow.setHeight(ARROW_HEIGHT);
            comp_data.ma.displayMarker(arrow);
        }
    }

    if (res.has_value()) {
        return this->k_obstacle.val(comp_data) * res.value();
    } else {
        return std::nullopt;
    }
}
}  // namespace luhsoccer::robot_control