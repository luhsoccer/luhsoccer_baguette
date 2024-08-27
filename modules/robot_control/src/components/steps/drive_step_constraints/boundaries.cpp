#include "boundaries.hpp"
#include "config/robot_control_config.hpp"
#include "robot_control/components/component_data.hpp"
#include "robot_control/components/features/obstacle_feature.hpp"
#include "robot_control/components/shapes/inverse_rectangle_shape.hpp"

namespace luhsoccer::robot_control {

std::vector<std::shared_ptr<const AbstractObstacle>> BoundariesConstraint ::getAdditionalObstacleFeaturesImpl() const {
    auto margin = [](const std::shared_ptr<const transform::WorldModel>& wm) {
        auto field_data = wm->getFieldData();
        return field_data.field_runoff_width - robotControlConfig().robot_radius;
    };

    auto width = [margin](const ComponentData& comp_data, const ComponentUid&) {
        auto field_data = comp_data.wm->getFieldData();
        return field_data.size.y() + 2 * margin(comp_data.wm);
    };

    auto height = [margin](const ComponentData& comp_data, const ComponentUid&) {
        auto field_data = comp_data.wm->getFieldData();
        return field_data.size.x() + 2 * margin(comp_data.wm);
    };
    auto obstacle = std::make_shared<ObstacleFeature>(InverseRectangleShape("", {CALLBACK, width}, {CALLBACK, height}));
    obstacle->setAvoidCollision(false);
    obstacle->setKObstacle(0.0);
    return {obstacle};
}

}  // namespace luhsoccer::robot_control