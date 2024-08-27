#include "ball_placement.hpp"
#include "robot_control/components/component_data.hpp"
#include "config/robot_control_config.hpp"
#include "robot_control/components/features/obstacle_feature.hpp"
#include "robot_control/components/shapes/circle_shape.hpp"
#include "robot_control/components/shapes/rectangle_shape.hpp"

namespace luhsoccer::robot_control {

bool BallPlacementConstraint::conditionMeetImpl(const ComponentData& comp_data) const {
    auto game_state = comp_data.wm->getGameState();
    bool is_enemy_ball_placement = game_state.value() == transform::GameState::BALL_PLACEMENT_ENEMY;
    bool is_ally_ball_placement = game_state.value() == transform::GameState::BALL_PLACEMENT_FORCE_START ||
                                  game_state.value() == transform::GameState::BALL_PLACEMENT_FREE_KICK;
    auto goalie = comp_data.wm->getGoalieID();
    bool robot_is_goalie = (goalie.has_value() && goalie.value() == comp_data.td.robot);
    return is_enemy_ball_placement || (is_ally_ball_placement && !robot_is_goalie);
}

std::vector<std::shared_ptr<const AbstractObstacle>> BallPlacementConstraint::getAdditionalObstacleFeaturesImpl()
    const {
    DoubleComponentParam radius(CALLBACK, []() -> double {
        return robotControlConfig().stop_state_min_ball_dist + robotControlConfig().robot_radius;
    });

    DoubleComponentParam width(CALLBACK, []() -> double {
        return (robotControlConfig().stop_state_min_ball_dist + robotControlConfig().robot_radius) * 2;
    });

    DoubleComponentParam height(CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> double {
        auto ball_pos = transform::Position("ball").getCurrentPosition(comp_data.wm, {BALL_PLACEMENT_POSITION_FRAME},
                                                                       comp_data.time);
        if (!ball_pos.has_value()) return 1.0;
        return (ball_pos->translation()).norm();
    });

    ComponentPosition rect_center(
        CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> transform::Position {
            auto ball_pos = transform::Position("ball").getCurrentPosition(comp_data.wm, "", comp_data.time);
            auto placement_pos =
                transform::Position(BALL_PLACEMENT_POSITION_FRAME).getCurrentPosition(comp_data.wm, "", comp_data.time);
            if (!ball_pos.has_value() || !placement_pos.has_value()) return {""};
            Eigen::Vector2d translation_mid = (ball_pos->translation() + placement_pos->translation()) / 2;
            Eigen::Vector2d diff = ball_pos->translation() - placement_pos->translation();
            double rotation = std::atan2(diff.y(), diff.x());
            return {"", translation_mid.x(), translation_mid.y(), rotation};
        });

    return {std::make_shared<ObstacleFeature>(CircleShape({"ball"}, radius, true)),
            std::make_shared<ObstacleFeature>(CircleShape({BALL_PLACEMENT_POSITION_FRAME}, radius, true)),
            std::make_shared<ObstacleFeature>(RectangleShape(rect_center, width, height, true))};
}
}  // namespace luhsoccer::robot_control