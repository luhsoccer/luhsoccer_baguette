#include "stop_state.hpp"

#include "local_planner_components/features/anti_target_feature.hpp"
#include "local_planner_components/features/robot_cf_obstacle.hpp"
#include "local_planner_components/shapes/point_shape.hpp"
#include "local_planner_components/shapes/rectangle_shape.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "transform/transform.hpp"

namespace luhsoccer::local_planner {

bool StopStateConstraint::conditionMeetImpl(const std::shared_ptr<const transform::WorldModel>& wm,
                                            const TaskData& /*td*/) const {
    if (localPlannerConfig().stop_state) return true;
    auto game_state = wm->getGameState();
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

[[nodiscard]] std::vector<std::shared_ptr<const AbstractTargetFeature>>
StopStateConstraint::getAdditionalTargetFeaturesImpl() const {
    const auto& config = localPlannerConfig();

    std::vector<std::shared_ptr<const AbstractTargetFeature>> target_features;

    DoubleComponentParam w_border(
        CALLBACK, [&config](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            auto ball_pos = wm->getTransform("ball");
            auto robot_to_ball = wm->getTransform("ball", td.robot.getFrame());
            const auto field_data = wm->getFieldData();
            if (ball_pos.has_value() && robot_to_ball.has_value() &&
                (std::abs(ball_pos->transform.translation().x()) >
                     field_data.size.x() / 2 + field_data.field_runoff_width - config.stop_state_min_ball_distance -
                         config.robot_radius * 2 ||
                 std::abs(ball_pos->transform.translation().y()) >
                     field_data.size.y() / 2 + field_data.field_runoff_width - config.stop_state_min_ball_distance -
                         config.robot_radius * 2) &&
                robot_to_ball->transform.translation().norm() <
                    config.stop_state_min_ball_distance + config.robot_radius) {
                return 100.0;
            } else {
                return 0.0;
            }
        });
    DoubleComponentParam w_ball(
        CALLBACK, [w_border](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            if (w_border.val(wm, td) > 1.0) {
                return 1.0;
            } else {
                return 100.0;
            }
        });
    DoubleComponentParam ball_anti_target_influence_distance(
        CALLBACK, [&config]() { return config.stop_state_min_ball_distance + config.robot_radius; });
    target_features.push_back(
        std::make_shared<AntiTargetFeature>(PointShape("ball"), ball_anti_target_influence_distance, w_ball));

    DoubleComponentParam border_anti_target_influence_distance(
        CALLBACK, [&config]() { return config.stop_state_min_ball_distance * 2 + config.robot_radius * 3; });
    target_features.push_back(std::make_shared<AntiTargetFeature>(
        RectangleShape({transform::field::CENTER},
                       {CALLBACK,
                        [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
                            auto field_data = wm->getFieldData();
                            return field_data.size.y() + 2 * field_data.field_runoff_width;
                        }},
                       {CALLBACK,
                        [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
                            auto field_data = wm->getFieldData();
                            return field_data.size.x() + 2 * field_data.field_runoff_width;
                        }},
                       false),
        border_anti_target_influence_distance, w_border));
    return target_features;
}

[[nodiscard]] std::vector<std::shared_ptr<const AbstractCFObstacle>>
StopStateConstraint::getAdditionalObstacleFeaturesImpl() const {
    const auto& config = localPlannerConfig();
    std::vector<std::shared_ptr<const AbstractCFObstacle>> obstacle_features;
    DoubleComponentParam w_b(
        CALLBACK, [&config](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            auto robot_pos = wm->getTransform("ball", td.robot.getFrame());
            if (robot_pos.has_value() &&
                robot_pos->transform.translation().norm() > config.stop_state_min_ball_distance + config.robot_radius) {
                return 1.0;
            } else {
                return 0.0;
            }
        });
    obstacle_features.push_back(std::make_shared<const RobotCFObstacle>(
        CircleShape({"ball"}, config.stop_state_min_ball_distance, false), w_b));
    return obstacle_features;
}

}  // namespace luhsoccer::local_planner