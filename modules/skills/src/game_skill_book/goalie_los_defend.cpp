#include "skill_books/game_skill_book/goalie_los_defend.hpp"
#include "config/skills_config.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/circle_shape.hpp"
#include "robot_control/components/shapes/line_shape.hpp"
#include "robot_control/components/steps/drive_step.hpp"
// include components here

namespace luhsoccer::skills {

GoalieLosDefendBuild::GoalieLosDefendBuild()
    : SkillBuilder("GoalieLosDefend",       //
                   {"Enemy ball carrier"},  //
                   {},                      //
                   {},                      //
                   {},                      //
                   {},                      //
                   {}){};

void GoalieLosDefendBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    DriveStep defend_goal;
    defend_goal.setRotationControl(HeadingRotationControl("ball"));
    defend_goal.setAvoidOtherRobots(false);
    defend_goal.setAvoidDefenseArea(false);
    defend_goal.setReachCondition(DriveStep::ReachCondition::NEVER);

    auto calc_y_on_goal = [](const std::shared_ptr<const transform::WorldModel>& wm,
                             const TaskData& td) -> std::optional<double> {
        auto b_pos = transform::Position("ball").getCurrentPosition(wm, transform::field::GOAL_ALLY_CENTER);
        auto e_pos = transform::Position(td.related_robots[0].getFrame())
                         .getCurrentPosition(wm, transform::field::GOAL_ALLY_CENTER);

        if (!b_pos.has_value() || !e_pos.has_value()) return std::nullopt;

        // if ((b_pos->translation() - e_pos->translation()).norm() >
        //     cs.skills_config.defend_goal_on_circle_enemy_threshold)
        //     return std::nullopt;

        double dx = b_pos->translation().x() - e_pos->translation().x();
        double dy = b_pos->translation().y() - e_pos->translation().y();

        return b_pos->translation().y() - ((dy / dx) * b_pos->translation().x());
    };
    ComponentPosition y_on_goal_line_point(
        CALLBACK,
        [&cs, calc_y_on_goal](const std::shared_ptr<const transform::WorldModel>& wm,
                              const TaskData& td) -> transform::Position {
            auto y_on_goal = calc_y_on_goal(wm, td);
            if (!y_on_goal.has_value()) return {"", 0.0, 0.0};
            auto field_data = wm->getFieldData();
            double half_goal_width = field_data.goal_width / 2 - cs.skills_config.goalie_defend_goal_margin;
            y_on_goal.value() =
                std::min(half_goal_width, std::max(-half_goal_width, y_on_goal.value()));  /// @todo use field data
            return {transform::field::GOAL_ALLY_CENTER, 0.0, y_on_goal.value()};
        });

    TargetFeature line_target(
        LineShape(y_on_goal_line_point, {"ball"}, cs.skills_config.goalie_defend_on_circle_cutoff));
    line_target.setIgnoreVelocity(true);
    defend_goal.addFeature(std::move(line_target));

    ComponentPosition circle_center(CALLBACK, [&cs]() -> transform::Position {
        return {transform::field::GOAL_ALLY_CENTER, -cs.skills_config.goalie_defend_on_circle_offset};
    });

    TargetFeature circle_target(CircleShape(circle_center, cs.skills_config.goalie_defend_on_circle_radius, false));
    circle_target.setWeight(cs.skills_config.goalie_defend_on_circle_weight);
    defend_goal.addFeature(std::move(circle_target));

    addStep(std::move(defend_goal));
    // end of skill
}
}  // namespace luhsoccer::skills