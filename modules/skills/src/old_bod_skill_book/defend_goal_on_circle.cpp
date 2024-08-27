#include "skill_books/bod_skill_book/defend_goal_on_circle.hpp"
#include "local_planner_components/features/ball_target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "config/skills_config.hpp"
// include components here

// States:
// intercept ball
// -> wenn ball auf tor geschossen wird

// Defend on circle : ball carrier heading
// -> wenn ball carrier == enemy
// -> wenn ball carrier auf goal guckt

// defend on circle : ball to goal line
// -> sonst

// steps

// DriveStep: defend goal
//  - CircleShape
//  - LineShape Heading -> nur aktiv wenn robot auf goal guckt
//  - LineShape ball zu tor mittelpunkt -> sonst aktiv
//  - cancel coniditon -> wenn ball auf tor geschossen wird

// dribbler anmachen?

// DriveStep: intercept ball
//

// ende
namespace luhsoccer::skills {

DefendGoalOnCircleBuild::DefendGoalOnCircleBuild()
    : SkillBuilder("DefendGoalOnCircle",  //
                   {},                    //
                   {},                    //
                   {},                    //
                   {},                    //
                   {},                    //
                   {}){};

void DefendGoalOnCircleBuild::buildImpl(const config_provider::ConfigStore& cs) {
    addStep(DribblerStep(robot_interface::DribblerMode::OFF));

    // // Defend the goal
    DriveStep defend_goal;
    defend_goal.setRotationControl(HeadingRotationControl("ball"));
    defend_goal.setAvoidOtherRobots(false);
    defend_goal.setAvoidDefenseArea(false);
    defend_goal.setReachCondition(DriveStep::ReachCondition::NEVER);

    // Find enemy ball carrier
    ComponentPosition enemy_ball_carrier(
        CALLBACK,
        [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> transform::Position {
            if (wm->getVisibleRobots<Team::ENEMY>().size() == 0) return {"", 0.0, 0.0};
            RobotIdentifier closest_robot = EMPTY_IDENTIFIER;
            for (const auto& robot : wm->getVisibleRobots<Team::ENEMY>()) {
                ComponentPosition r_comp{robot.getFrame()};

                auto r_pos = r_comp.positionObject(wm, td).getCurrentPosition(wm, "ball");

                if (!r_pos.has_value()) continue;

                if (closest_robot == EMPTY_IDENTIFIER) {
                    closest_robot = robot;
                    continue;
                }

                auto closest_robot_pos =
                    ComponentPosition(closest_robot.getFrame()).positionObject(wm, td).getCurrentPosition(wm, "ball");
                if (!closest_robot_pos.has_value()) continue;

                if (r_pos->translation().norm() < closest_robot_pos->translation().norm()) {
                    closest_robot = robot;
                }
            }
            if (closest_robot == EMPTY_IDENTIFIER) return {"", 0.0, 0.0};
            return {closest_robot.getFrame()};
        });

    // Weight calculation

    auto calc_y_on_goal = [&cs, enemy_ball_carrier](const std::shared_ptr<const transform::WorldModel>& wm,
                                                    const TaskData& td) -> std::optional<double> {
        ComponentPosition b_comp("ball");

        auto b_pos = b_comp.positionObject(wm, td).getCurrentPosition(wm, transform::field::GOAL_ALLY_CENTER);
        auto e_pos =
            enemy_ball_carrier.positionObject(wm, td).getCurrentPosition(wm, transform::field::GOAL_ALLY_CENTER);

        if (!b_pos.has_value() || !e_pos.has_value()) return std::nullopt;

        if ((b_pos->translation() - e_pos->translation()).norm() >
            cs.skills_config.defend_goal_on_circle_enemy_threshold)
            return std::nullopt;

        double dx = b_pos->translation().x() - e_pos->translation().x();
        double dy = b_pos->translation().y() - e_pos->translation().y();

        return b_pos->translation().y() - ((dy / dx) * b_pos->translation().x());
    };

    DoubleComponentParam weight(
        CALLBACK,
        [&cs, calc_y_on_goal](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            // LOG_INFO(logger::Logger("DefendGOalONCircle"), "y_on_goal_line: {:0.3f}", y_on_goal_line);
            auto y_on_goal_line = calc_y_on_goal(wm, td);
            if (!y_on_goal_line.has_value()) return 0.0;
            if (std::abs(y_on_goal_line.value()) < cs.skills_config.defend_goal_on_circle_threshold)
                return cs.skills_config.defend_goal_on_circle_line_weight;
            return 0.0;
        });
    DoubleComponentParam weight_negated(
        CALLBACK, [&cs, weight](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
            if (weight.val(wm, td) > 0.5) return 0.0;
            return cs.skills_config.defend_goal_on_circle_line_weight;
        });

    ComponentPosition y_on_goal_line_point(
        CALLBACK,
        [&cs, calc_y_on_goal](const std::shared_ptr<const transform::WorldModel>& wm,
                              const TaskData& td) -> transform::Position {
            auto y_on_goal = calc_y_on_goal(wm, td);
            if (!y_on_goal.has_value()) return {"", 0.0, 0.0};
            double half_goal_width = cs.skills_config.half_goal_width;
            y_on_goal.value() =
                std::min(half_goal_width, std::max(-half_goal_width, y_on_goal.value()));  /// @todo use field data
            return {transform::field::GOAL_ALLY_CENTER, 0.0, y_on_goal.value()};
        });

    // NOLINTNEXTLINE(cppcoreguidelines - avoid - magic - numbers)
    defend_goal.addFeature(
        TargetFeature(LineShape(y_on_goal_line_point, "ball", cs.skills_config.defend_goal_on_circle_line_goal_cutoff),
                      cs.skills_config.translational_tolerance, weight, true));

    // Move to line/circle
    ComponentPosition goal_center(transform::field::GOAL_ALLY_CENTER);
    ComponentPosition ball_pos("ball");
    ComponentPosition circle_center(
        CALLBACK,
        [&cs](const std::shared_ptr<const transform::WorldModel>& /*wm*/,
              const TaskData& /*td*/) -> transform::Position {
            return {transform::field::GOAL_ALLY_CENTER, -cs.skills_config.defend_goal_on_circle_offset};
        });
    defend_goal.addFeature(
        TargetFeature(LineShape(ball_pos, goal_center, 0.0, cs.skills_config.defend_goal_on_circle_line_goal_cutoff),
                      cs.skills_config.translational_tolerance, weight_negated, true));

    defend_goal.addFeature(
        TargetFeature(CircleShape(circle_center, cs.skills_config.defend_goal_on_circle_radius, false),
                      cs.skills_config.translational_tolerance, cs.skills_config.defend_goal_on_circle_weight, true));

    addStep(std::move(defend_goal));
    // end of skill
}
}  // namespace luhsoccer::skills