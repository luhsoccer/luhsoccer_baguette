#include "skill_books/bod_skill_book/prepare_kick.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/steps/turn_around_ball_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
// include components here

namespace luhsoccer::skills {

PrepareKickBuild::PrepareKickBuild()
    : SkillBuilder("PrepareKick",    //
                   {},               //
                   {"TargetPoint"},  //
                   {},               //
                   {},               //
                   {},               //
                   {}){};

void PrepareKickBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    // Drive to ball
    DriveStep drive_to_ball;

    drive_to_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    drive_to_ball.setAvoidOtherRobots(true);
    ComponentPosition heading(TD_Pos::POINT, 0);
    drive_to_ball.setRotationControl(HeadingRotationControl({TD_Pos::POINT, 0}));

    drive_to_ball.addFeature(TargetFeature(CircleShape("ball", 0.3, false)));

    drive_to_ball.setMaxVelX(cs.skills_config.prepare_kick_max_vel_x);
    drive_to_ball.setMaxVelY(cs.skills_config.prepare_kick_max_vel_y);

    addStep(std::move(drive_to_ball));

    // Turn around ball to face target
    ComponentPosition target_pos(TD_Pos::POINT, 0);
    TurnAroundBallStep turn_to_target(target_pos);

    turn_to_target.setAvoidOtherRobots(true);
    turn_to_target.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);

    addStep(std::move(turn_to_target));
    // end of skill
}
}  // namespace luhsoccer::skills