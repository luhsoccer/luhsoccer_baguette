#include "skill_books/bod_skill_book/block_goalie_lo_s.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/shapes/compose_shape.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/steps/kick_step.hpp"
// include components here

namespace luhsoccer::skills {

BlockGoalieLoSBuild::BlockGoalieLoSBuild()
    : SkillBuilder("BlockGoalieLoS",      //
                   {"EnemyBallCarrier"},  //
                   {},                    //
                   {},                    //
                   {},                    //
                   {},                    //
                   {}){};

void BlockGoalieLoSBuild::buildImpl(const config_provider::ConfigStore& cs) {
    addStep(DribblerStep(robot_interface::DribblerMode::HIGH));
    addStep(KickStep(6.5, false, robot_interface::KickExecuteTime::WHEN_BALL_IN_DRIBBLER));

    DriveStep drive_to_line_of_sight;

    drive_to_line_of_sight.setReachCondition(DriveStep::ReachCondition::NEVER);
    drive_to_line_of_sight.setAvoidOtherRobots(true);

    drive_to_line_of_sight.setRotationControl(HeadingRotationControl("ball"));

    // LineShape goalie to ball
    drive_to_line_of_sight.addFeature(
        TargetFeature(LineShape("ball", {TD_Pos::ROBOT, 0}, -3.0, 0.0), cs.skills_config.translational_tolerance, 1.0));

    ComponentPosition goal_center{transform::field::GOAL_ENEMY_CENTER};
    drive_to_line_of_sight.addFeature(
        TargetFeature(CircleShape(goal_center, 1.75, false), cs.skills_config.translational_tolerance, 1.0));

    addStep(std::move(drive_to_line_of_sight));

    // end of skill
}
}  // namespace luhsoccer::skills
