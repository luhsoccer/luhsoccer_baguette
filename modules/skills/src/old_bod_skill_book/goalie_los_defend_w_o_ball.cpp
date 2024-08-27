#include "skill_books/bod_skill_book/goalie_los_defend_w_o_ball.hpp"
#include "config/skills_config.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "config_provider/config_store_main.hpp"

// include components here

namespace luhsoccer::skills {

GoalieLosDefendWOBallBuild::GoalieLosDefendWOBallBuild()
    : SkillBuilder("goalieLosDefendWOBall",                       //
                   {"Enemy ball carrier"},                        //
                   {},                                            //
                   {},                                            //
                   {},                                            //
                   {"Prevent ReflexKick", "Limit to Ball half"},  //
                   {}){};

void GoalieLosDefendWOBallBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step

    DriveStep defend_goal;
    defend_goal.setRotationControl(HeadingRotationControl({TD_Pos::ROBOT, 0}));
    defend_goal.setAvoidOtherRobots(false);
    defend_goal.setAvoidDefenseArea(false);
    defend_goal.setReachCondition(DriveStep::ReachCondition::NEVER);

    auto calc_y_on_goal = [&cs](const std::shared_ptr<const transform::WorldModel>& wm,
                                const TaskData& td) -> std::optional<double> {
        auto e_pos = transform::Position(td.related_robots[0].getFrame())
                         .getCurrentPosition(wm, transform::field::GOAL_ALLY_CENTER);

        if (!e_pos.has_value()) return std::nullopt;

        double heading_offset = 0.0;
        double enemy_heading = Eigen::Rotation2Dd(e_pos->rotation()).angle();
        if (td.required_bools[0]) {
            // add incoming angle to outgoing angle
            auto ball_pos = transform::Position("ball").getCurrentPosition(wm, transform::field::GOAL_ALLY_CENTER);
            if (!ball_pos.has_value()) return std::nullopt;
            Eigen::Vector2d robot_to_ball = ball_pos->translation() - e_pos->translation();
            double ball_robot_heading = atan2(robot_to_ball.y(), robot_to_ball.x());
            heading_offset = cropAngle(enemy_heading - ball_robot_heading) *
                             cs.skills_config.goalie_los_defend_wo_ball_reflex_angle_scaling;
        }
        double y_coordinate =
            e_pos->translation().y() - tan(enemy_heading - L_PI + heading_offset) * e_pos->translation().x();

        if (td.required_bools[1]) {
            // limit to half of ball
            auto ball_pos = transform::Position("ball").getCurrentPosition(wm, transform::field::GOAL_ALLY_CENTER);
            if (!ball_pos.has_value()) return std::nullopt;
            if (ball_pos->translation().y() > 0) {
                y_coordinate = std::max(y_coordinate, 0.0);
            } else {
                y_coordinate = std::min(y_coordinate, 0.0);
            }
        }
        return y_coordinate;
    };
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

    defend_goal.addFeature(TargetFeature(
        LineShape(y_on_goal_line_point, {TD_Pos::ROBOT, 0}, cs.skills_config.defend_goal_on_circle_line_goal_cutoff),
        cs.skills_config.translational_tolerance, cs.skills_config.defend_goal_on_circle_line_weight, true));

    ComponentPosition circle_center(
        CALLBACK,
        [&cs](const std::shared_ptr<const transform::WorldModel>& /*wm*/,
              const TaskData& /*td*/) -> transform::Position {
            return {transform::field::GOAL_ALLY_CENTER, -cs.skills_config.defend_goal_on_circle_offset};
        });

    defend_goal.addFeature(
        TargetFeature(CircleShape(circle_center, cs.skills_config.defend_goal_on_circle_radius, false),
                      cs.skills_config.translational_tolerance, cs.skills_config.defend_goal_on_circle_weight, true));

    addStep(std::move(defend_goal));
}
}  // namespace luhsoccer::skills