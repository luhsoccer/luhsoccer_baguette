#include "enemy_kickoff.hpp"
#include "robot_control/components/component_data.hpp"
#include "config/robot_control_config.hpp"
#include "robot_control/components/features/obstacle_feature.hpp"
#include "robot_control/components/shapes/circle_shape.hpp"
#include "robot_control/components/shapes/rectangle_shape.hpp"

namespace luhsoccer::robot_control {

bool EnemyKickOffConstraint::conditionMeetImpl(const ComponentData& comp_data) const {
    auto game_state = comp_data.wm->getGameState();
    return game_state.value() == transform::GameState::KICKOFF_ENEMY ||
           game_state.value() == transform::GameState::KICKOFF_PREP_ENEMY;
}

std::vector<std::shared_ptr<const AbstractObstacle>> EnemyKickOffConstraint::getAdditionalObstacleFeaturesImpl() const {
    DoubleComponentParam center_radius(CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> double {
        auto field_data = comp_data.wm->getFieldData();
        return field_data.center_circle_radius + robotControlConfig().robot_radius;
    });

    DoubleComponentParam field_height(CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> double {
        auto field_data = comp_data.wm->getFieldData();
        return field_data.size.y() + field_data.field_runoff_width * 2;
    });
    DoubleComponentParam field_half_width(CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> double {
        auto field_data = comp_data.wm->getFieldData();
        return field_data.size.x() / 2 + robotControlConfig().robot_radius + field_data.field_runoff_width;
    });
    ComponentPosition enemy_half_center(
        CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> transform::Position {
            auto field_data = comp_data.wm->getFieldData();
            return {"",
                    field_data.size.x() / 4 - robotControlConfig().robot_radius / 2 + field_data.field_runoff_width / 2,
                    0.0, 0.0};
        });

    return {std::make_shared<ObstacleFeature>(CircleShape({""}, center_radius, true)),
            std::make_shared<ObstacleFeature>(RectangleShape(enemy_half_center, field_height, field_half_width, true))};
}
}  // namespace luhsoccer::robot_control