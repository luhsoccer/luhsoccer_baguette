#include "stop_state_constraint.hpp"
#include "robot_control/components/component_data.hpp"
#include "config/robot_control_config.hpp"
#include "robot_control/components/features/obstacle_feature.hpp"
#include "robot_control/components/shapes/circle_shape.hpp"

namespace luhsoccer::robot_control {

bool StopStateConstraint::conditionMeetImpl(const ComponentData& comp_data) const {
    auto game_state = comp_data.wm->getGameState();
    static const std::vector<transform::GameState> STOP_STATES{transform::GameState::STOP,
                                                               transform::GameState::FREE_KICK_ENEMY,
                                                               transform::GameState::PENALTY_PREP_ENEMY,
                                                               transform::GameState::KICKOFF_ENEMY,
                                                               transform::GameState::KICKOFF_PREP_ENEMY,
                                                               transform::GameState::KICKOFF_PREP};
    if (game_state.has_value())
        return std::find(STOP_STATES.begin(), STOP_STATES.end(), game_state.value()) != STOP_STATES.end();
    else
        return false;
}

std::vector<std::shared_ptr<const AbstractObstacle>> StopStateConstraint::getAdditionalObstacleFeaturesImpl() const {
    DoubleComponentParam radius(CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> double {
        double free_kick_margin = 0.0;
        auto game_state = comp_data.wm->getGameState();
        if (game_state.has_value() && (game_state == transform::GameState::FREE_KICK_ENEMY ||
                                       game_state == transform::GameState::FREE_KICK_PREP_ENEMY)) {
            free_kick_margin = robotControlConfig().stop_state_free_kick_margin;
        }
        return robotControlConfig().stop_state_min_ball_dist + robotControlConfig().robot_radius - free_kick_margin;
    });
    return {
        std::make_shared<ObstacleFeature>(CircleShape({"ball"}, radius, true)),
    };
}
}  // namespace luhsoccer::robot_control