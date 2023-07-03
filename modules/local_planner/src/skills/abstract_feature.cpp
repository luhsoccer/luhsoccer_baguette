

#include "local_planner/skills/abstract_feature.hpp"

#include "local_planner/skills/abstract_shape.hpp"
#include "config_provider/config_store_main.hpp"
namespace luhsoccer::local_planner {

std::vector<Marker> AbstractFeature::getVisualizationMarker(const std::shared_ptr<const transform::WorldModel>& wm,
                                                            const TaskData& td, const RobotIdentifier& robot,
                                                            time::TimePoint time) const {
    const auto& viz_options = config_provider::ConfigProvider::getConfigStore().local_planner_visualization_config;
    std::vector<Marker> ms;
    auto color = this->getVisualizationColor(wm, td, robot, time);
    if (viz_options.shapes_display) {
        std::vector<Marker> ms_shape = this->shape->getVisualizationMarker(wm, td, time);
        for (auto& m : ms_shape) {
            auto visitor = overload{[color](marker::Marker& marker) { marker.setColor(color); }};
            std::visit(visitor, m);
        }
        ms.insert(ms.end(), ms_shape.begin(), ms_shape.end());
    }
    if (viz_options.shapes_vector_display) {
        auto to_closest_point =
            this->shape->getTransformToClosestPoint(wm, td, {robot.getFrame()}, time, {robot.getFrame()});
        if (to_closest_point.vec.has_value()) {
            marker::Line shape_vec(robot.getFrame());
            shape_vec.setLinePoints({0.0, 0.0}, {to_closest_point.vec->x(), to_closest_point.vec->y()});
            shape_vec.setColor(color);
            shape_vec.setFrameLocked(true);
            ms.emplace_back(shape_vec);
        }
    }
    if (viz_options.shapes_vector_to_center_display) {
        auto to_center_vec = this->shape->getCenter(wm, td, time, {robot.getFrame()});
        if (to_center_vec.has_value()) {
            marker::Line shape_vec(robot.getFrame());
            shape_vec.setLinePoints({0.0, 0.0}, {to_center_vec->x(), to_center_vec->y()});
            shape_vec.setColor(color);
            shape_vec.setFrameLocked(true);
            ms.emplace_back(shape_vec);
        }
    }
    return ms;
}

std::optional<std::pair<Eigen::Vector2d, Eigen::Vector2d>> AbstractCFObstacle::getVecAndVelocity(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
    const time::TimePoint& time, const ComponentPosition& observe_position) const {
    auto robot_to_feature = this->shape->getTransformToClosestPoint(wm, td, robot, time, observe_position);
    std::pair<Eigen::Vector2d, Eigen::Vector2d> vec_and_vel;
    if (robot_to_feature.vec.has_value()) {
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

}  // namespace luhsoccer::local_planner