#include "ball_placement.hpp"

#include "local_planner_components/features/robot_cf_obstacle.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/shapes/rectangle_shape.hpp"
#include "local_planner_components/shapes/compose_shape.hpp"
#include "local_planner_components/complex_positions.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/features/anti_target_feature.hpp"

namespace luhsoccer::local_planner {

BallPlacement::BallPlacement()
    : DriveStepConstraint(),
      weight(CALLBACK, [this](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
          const static LineShape LINE("ball", {this->BALL_PLACEMENT_POSITION_FRAME});
          auto res = LINE.getTransformToClosestPoint(wm, td, td.robot);
          if (!res.vec.has_value()) return 0.0;

          if (res.vec->norm() < localPlannerConfig().stop_state_min_ball_distance) {
              return 0.0;
          } else {
              return 1.0;
          }
      }) {
    this->initFeatures();
};

bool BallPlacement::conditionMeetImpl(const std::shared_ptr<const transform::WorldModel>& wm,
                                      const TaskData& /*td*/) const {
    auto game_state = wm->getGameState();
    return game_state.has_value() && (game_state.value() == transform::GameState::BALL_PLACEMENT_ENEMY);
}

std::vector<std::shared_ptr<const AbstractTargetFeature>> BallPlacement::getAdditionalTargetFeaturesImpl() const {
    DoubleComponentParam anti_weight(
        CALLBACK, [this](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            const auto& config = localPlannerConfig();
            auto ball_pos = wm->getTransform("ball");
            auto placement_pos = wm->getTransform(this->BALL_PLACEMENT_POSITION_FRAME);
            const static LineShape LINE("ball", {this->BALL_PLACEMENT_POSITION_FRAME});

            auto robot_to_line = LINE.getTransformToClosestPoint(wm, td, td.robot);
            const auto field_data = wm->getFieldData();

            const auto close_to_border = [&field_data, &config](Eigen::Affine2d position) -> bool {
                return std::abs(position.translation().x()) > field_data.size.x() / 2 + field_data.field_runoff_width -
                                                                  config.stop_state_min_ball_distance -
                                                                  config.robot_radius * 2 ||
                       std::abs(position.translation().y()) > field_data.size.y() / 2 + field_data.field_runoff_width -
                                                                  config.stop_state_min_ball_distance -
                                                                  config.robot_radius * 2;
            };
            if (ball_pos.has_value() && placement_pos.has_value() && robot_to_line.vec.has_value() &&
                robot_to_line.vec->norm() < config.stop_state_min_ball_distance + config.robot_radius &&
                (close_to_border(ball_pos->transform) || close_to_border(placement_pos->transform))) {
                return 0.0;
            } else {
                return 100.0;
            }
        });
    DoubleComponentParam border_weight(
        CALLBACK, [this, anti_weight](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) {
            const static LineShape LINE("ball", {this->BALL_PLACEMENT_POSITION_FRAME});
            const auto& config = localPlannerConfig();

            auto robot_to_line = LINE.getTransformToClosestPoint(wm, td, td.robot);
            if (anti_weight.val(wm, td) > 1.0 / 2) {
                return 0.0;
            } else if (robot_to_line.vec.has_value() &&
                       robot_to_line.vec->norm() < config.stop_state_min_ball_distance + config.robot_radius) {
                return 100.0;
            } else {
                return 0.0;
            }
        });
    DoubleComponentParam border_anti_target_influence_distance(CALLBACK, []() {
        const auto& config = localPlannerConfig();
        return config.stop_state_min_ball_distance * 2 + config.robot_radius * 3;
    });

    return {std::make_shared<AntiTargetFeature>(LineShape("ball", {this->BALL_PLACEMENT_POSITION_FRAME}),
                                                localPlannerConfig().stop_state_min_ball_distance, anti_weight),
            std::make_shared<AntiTargetFeature>(
                RectangleShape(
                    {transform::field::CENTER},
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
                border_anti_target_influence_distance, border_weight)};
}

std::vector<std::shared_ptr<const AbstractCFObstacle>> BallPlacement::getAdditionalObstacleFeaturesImpl() const {
    ComposeShape c;
    c.addShape(CircleShape("ball", localPlannerConfig().stop_state_min_ball_distance, false));
    c.addShape(
        CircleShape({this->BALL_PLACEMENT_POSITION_FRAME}, localPlannerConfig().stop_state_min_ball_distance, false));
    ComponentPosition rect_center(
        CALLBACK,
        [this](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> transform::Position {
            auto ball_pos = wm->getTransform("ball");
            auto placement_pos = wm->getTransform(this->BALL_PLACEMENT_POSITION_FRAME);
            if (!ball_pos.has_value() || !placement_pos.has_value()) return {""};
            Eigen::Vector2d translation_mid =
                (ball_pos->transform.translation() + placement_pos->transform.translation()) / 2;
            Eigen::Vector2d diff = ball_pos->transform.translation() - placement_pos->transform.translation();
            double rotation = std::atan2(diff.y(), diff.x());
            return {"", translation_mid.x(), translation_mid.y(), rotation};
        });

    DoubleComponentParam len_rect(
        CALLBACK, [this](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
            auto ball_pos = wm->getTransform("ball");
            auto placement_pos = wm->getTransform(this->BALL_PLACEMENT_POSITION_FRAME);
            if (!ball_pos.has_value() || !placement_pos.has_value()) return 1.0;
            return (ball_pos->transform.translation() - placement_pos->transform.translation()).norm();
        });

    c.addShape(RectangleShape(
        rect_center, {CALLBACK, []() -> double { return localPlannerConfig().stop_state_min_ball_distance * 2; }},
        len_rect, false));

    return {std::make_shared<RobotCFObstacle>(std::move(c), this->weight)};
}
}  // namespace luhsoccer::local_planner