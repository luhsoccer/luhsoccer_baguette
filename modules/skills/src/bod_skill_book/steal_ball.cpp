#include "skill_books/bod_skill_book/steal_ball.hpp"
// include components here
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/features/ball_target_feature.hpp"
#include "local_planner_components/steps/turn_around_ball_step.hpp"
#include "local_planner_components/steps/condition_step.hpp"

#include "bod_skill_book/move_to_ball_turn_radius_macro.hpp"

namespace luhsoccer::skills {

StealBallBuild::StealBallBuild()
    : SkillBuilder("StealBall",       //
                   {"ball_carrier"},  //
                   {},                //
                   {},                //
                   {},                //
                   {},                //
                   {}){};

void StealBallBuild::buildImpl(const config_provider::ConfigStore& cs) {
    ConditionStep c(
        {CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> bool {
             auto ball_info = wm->getBallInfo();
             if (ball_info.has_value()) {
                 if (ball_info->state == transform::BallState::IN_ROBOT && ball_info->robot.has_value() &&
                     ball_info->robot->isAlly()) {
                     return false;
                 }
             }
             return true;
         }});

    DriveStep go_to_get_ball;
    go_to_get_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    go_to_get_ball.setAvoidOtherRobots(true);
    go_to_get_ball.setRotationControl(HeadingRotationControl("ball"));
    go_to_get_ball.addFeature(TargetFeature(CircleShape("ball", cs.skills_config.move_to_ball_pre_radius, true)));
    c.addIfStep(std::move(go_to_get_ball));

    c.addIfStep(TurnAroundBallStep({TD_Pos::ROBOT, 0}));

    c.addIfStep(DribblerStep(robot_interface::DribblerMode::HIGH));

    DriveStep steal_ball;
    steal_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    steal_ball.setAvoidOtherRobots(false);
    steal_ball.setRotationControl(HeadingRotationControl("ball"));
    steal_ball.addFeature(BallTargetFeature(PointShape("ball")));
    c.addIfStep(std::move(steal_ball));

    DriveStep move_back;
    move_back.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    move_back.setAvoidOtherRobots(true);
    move_back.setRotationControl(HeadingRotationControl({TD_Pos::ROBOT, 0}));
    move_back.addFeature(TargetFeature(CircleShape({TD_Pos::ROBOT, 0}, 0.5, false)));
    move_back.setMaxVelX(1.0);
    move_back.setMaxVelY(1.0);
    c.addIfStep(std::move(move_back));

    addStep(std::move(c));
    // end of skill
}
}  // namespace luhsoccer::skills