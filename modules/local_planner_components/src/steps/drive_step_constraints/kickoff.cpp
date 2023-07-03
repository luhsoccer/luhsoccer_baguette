#include "kickoff.hpp"

#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/shapes/arc_shape.hpp"
#include "local_planner_components/shapes/compose_shape.hpp"
#include "local_planner_components/features/robot_cf_obstacle.hpp"

namespace luhsoccer::local_planner {

bool Kickoff::conditionMeetImpl(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) const {
    auto game_state = wm->getGameState();
    if (!game_state.has_value()) return false;
    return ((game_state.value() == transform::GameState::KICKOFF_ENEMY ||
             game_state.value() == transform::GameState::KICKOFF_PREP_ENEMY) &&
            this->enemy) ||
           ((game_state.value() == transform::GameState::KICKOFF ||
             game_state.value() == transform::GameState::KICKOFF_PREP) &&
            !this->enemy);
}

std::vector<std::shared_ptr<const AbstractCFObstacle>> Kickoff::getAdditionalObstacleFeaturesImpl() const {
    constexpr double FIELD_RUNOFF_WIDTH = 0.3;
    LineShape l_top(
        {{transform::field::MID_LINE_LEFT}},
        {CALLBACK,
         [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> transform::Position {
             return {transform::field::CENTER, 0.0, wm->getFieldData().center_circle_radius};
         }},
        -FIELD_RUNOFF_WIDTH);
    LineShape l_bottom(
        {{transform::field::MID_LINE_RIGHT}},
        {CALLBACK,
         [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> transform::Position {
             return {transform::field::CENTER, 0.0, -wm->getFieldData().center_circle_radius};
         }},
        -FIELD_RUNOFF_WIDTH);
    ArcShape arc({{transform::field::CENTER, 0.0, 0.0, this->enemy ? L_PI : 0.0}},
                 {CALLBACK,
                  [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
                      return wm->getFieldData().center_circle_radius;
                  }},
                 L_PI);
    ComposeShape c;

    c.addShape(std::move(arc));
    c.addShape(std::move(l_top));
    c.addShape(std::move(l_bottom));
    bool enemy = this->enemy;
    DoubleComponentParam w(
        CALLBACK, [enemy](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            auto trans = wm->getTransform(td.robot.getFrame());
            if (enemy) {
                if (trans.has_value() &&
                    (trans->transform.translation().x() > 0.0 ||
                     trans->transform.translation().norm() < wm->getFieldData().center_circle_radius)) {
                    return 0.0;
                } else {
                    return 1.0;
                }
            } else {
                if (trans.has_value() &&
                    (trans->transform.translation().x() > 0.0 &&
                     trans->transform.translation().norm() > wm->getFieldData().center_circle_radius)) {
                    return 0.0;
                } else {
                    return 1.0;
                }
            }
        });
    return {std::make_unique<RobotCFObstacle>(std::move(c), w, std::nullopt, std::nullopt,
                                              localPlannerConfig().defense_area_collision_distance)};
}

}  // namespace luhsoccer::local_planner