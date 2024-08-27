#include "leave_field.hpp"
#include "config/robot_control_config.hpp"
#include "robot_control/components/component_data.hpp"
#include "robot_control/components/features/obstacle_feature.hpp"
#include "robot_control/components/shapes/inverse_rectangle_shape.hpp"

namespace luhsoccer::robot_control {

std::vector<std::shared_ptr<const AbstractObstacle>> LeaveFieldConstraint ::getAdditionalObstacleFeaturesImpl() const {
    auto margin = []() { return -robotControlConfig().robot_radius; };

    auto width = [margin](const ComponentData& comp_data, const ComponentUid&) {
        auto field_data = comp_data.wm->getFieldData();
        return field_data.size.y() + 2 * margin();
    };

    auto height = [margin](const ComponentData& comp_data, const ComponentUid&) {
        auto field_data = comp_data.wm->getFieldData();
        return field_data.size.x() + 2 * margin();
    };
    auto obstacle = std::make_shared<ObstacleFeature>(InverseRectangleShape("", {CALLBACK, width}, {CALLBACK, height}));
    obstacle->setAvoidCollision(false);
    obstacle->setKObstacle(0.0);
    return {obstacle};
}

}  // namespace luhsoccer::robot_control