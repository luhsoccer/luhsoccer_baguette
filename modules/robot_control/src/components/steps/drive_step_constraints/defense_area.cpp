#include "defense_area.hpp"
#include "robot_control/components/component_data.hpp"
#include "config/robot_control_config.hpp"
#include "robot_control/components/features/obstacle_feature.hpp"
#include "robot_control/components/shapes/rectangle_shape.hpp"

namespace luhsoccer::robot_control {

bool DefenseAreaConstraint::conditionMeetImpl(const ComponentData& comp_data) const {
    auto goalie = comp_data.wm->getGoalieID();
    return ((!goalie.has_value() || goalie.value() != comp_data.td.robot) && robotControlConfig().avoid_defense_area);
}

[[nodiscard]] std::vector<std::shared_ptr<const AbstractObstacle>>
DefenseAreaConstraint::getAdditionalObstacleFeaturesImpl() const {
    auto margin = [](bool add_stop_state_margin) {
        return robotControlConfig().stop_state_defense_area_margin + robotControlConfig().robot_radius +
               (add_stop_state_margin ? robotControlConfig().stop_state_defense_area_margin_stop : 0.0);
    };
    auto is_stop_state = [](const std::shared_ptr<const transform::WorldModel>& wm) {
        auto game_state = wm->getGameState();
        return game_state.has_value() && (game_state.value() == transform::GameState::STOP ||
                                          game_state.value() == transform::GameState::FREE_KICK_ENEMY);
    };
    RectangleShape r1({{transform::field::GOAL_ENEMY_CENTER, FIELD_RUNOFF_WIDTH}},
                      {CALLBACK,
                       [margin, is_stop_state](const ComponentData& comp_data, const ComponentUid&) -> double {
                           return comp_data.wm->getFieldData().penalty_area_width +
                                  margin(is_stop_state(comp_data.wm)) * 2;
                       }},
                      {CALLBACK,
                       [margin, is_stop_state](const ComponentData& comp_data, const ComponentUid&) -> double {
                           return 2 * (comp_data.wm->getFieldData().penalty_area_depth + FIELD_RUNOFF_WIDTH +
                                       margin(is_stop_state(comp_data.wm)));
                       }},
                      true);

    RectangleShape r2(
        {{transform::field::GOAL_ALLY_CENTER, -FIELD_RUNOFF_WIDTH}},
        {CALLBACK,
         [margin](const ComponentData& comp_data, const ComponentUid&) -> double {
             return comp_data.wm->getFieldData().penalty_area_width + margin(false) * 2;
         }},
        {CALLBACK,
         [margin](const ComponentData& comp_data, const ComponentUid&) -> double {
             return 2 * (comp_data.wm->getFieldData().penalty_area_depth + FIELD_RUNOFF_WIDTH + margin(false));
         }},
        true);

    return {std::make_shared<ObstacleFeature>(std::move(r1)), std::make_shared<ObstacleFeature>(std::move(r2))};
}

}  // namespace luhsoccer::robot_control