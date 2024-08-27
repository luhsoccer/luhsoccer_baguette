#include "skill_books/game_skill_book/goalie_defend_on_circle.hpp"
#include "config/skills_config.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/circle_shape.hpp"
#include "robot_control/components/shapes/line_shape.hpp"
#include "robot_control/components/steps/dribbler_step.hpp"
#include "robot_control/components/steps/drive_step.hpp"
// include components here

namespace luhsoccer::skills {

GoalieDefendOnCircleBuild::GoalieDefendOnCircleBuild()
    : SkillBuilder("GoalieDefendOnCircle",  //
                   {},                      //
                   {},                      //
                   {},                      //
                   {},                      //
                   {},                      //
                   {}){};

void GoalieDefendOnCircleBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step

    addStep(DribblerStep(robot_interface::DribblerMode::OFF));

    // // Defend the goal
    DriveStep defend_goal;
    defend_goal.setRotationControl(HeadingRotationControl("ball"));
    defend_goal.setAvoidOtherRobots(false);
    defend_goal.setAvoidDefenseArea(false);
    defend_goal.setReachCondition(DriveStep::ReachCondition::NEVER);

    ComponentPosition circle_center(CALLBACK, [&cs]() -> transform::Position {
        return {transform::field::GOAL_ALLY_CENTER, -cs.skills_config.goalie_defend_on_circle_offset};
    });

    TargetFeature line_target(LineShape({"ball"}, {transform::field::GOAL_ALLY_CENTER}, 0.0,
                                        cs.skills_config.goalie_defend_on_circle_cutoff));
    line_target.setIgnoreVelocity(true);
    defend_goal.addFeature(std::move(line_target));

    TargetFeature circle_target(CircleShape(circle_center, cs.skills_config.goalie_defend_on_circle_radius, false));
    circle_target.setWeight(cs.skills_config.goalie_defend_on_circle_weight);
    defend_goal.addFeature(std::move(circle_target));

    addStep(std::move(defend_goal));
    // end of skill
}
}  // namespace luhsoccer::skills