#include "skill_books/bod_skill_book/kick_ball_through_target_direct.hpp"
// include components here
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/steps/wait_step.hpp"
#include "local_planner_components/steps/kick_step.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/features/ball_target_feature.hpp"
#include "local_planner_components/features/robot_cf_obstacle.hpp"
#include "local_planner_components/steps/turn_around_ball_step.hpp"

namespace luhsoccer::skills {

kickBallThroughTargetDirectBuild::kickBallThroughTargetDirectBuild()
    : SkillBuilder("kickBallThroughTargetDirect",  //
                   {},                             //
                   {"TargetPoint"},                //
                   {"KickVelocity"},               //
                   {},                             //
                   {},                             //
                   {}){};

void kickBallThroughTargetDirectBuild::buildImpl(const config_provider::ConfigStore& cs) {
    addStep(DribblerStep(robot_interface::DribblerMode::OFF));

    DriveStep go_to_get_ball;
    go_to_get_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    go_to_get_ball.setAvoidOtherRobots(true);
    go_to_get_ball.setRotationControl(HeadingRotationControl("ball"));
    go_to_get_ball.addFeature(TargetFeature(CircleShape("ball", cs.skills_config.move_to_ball_pre_radius, true)));
    addStep(std::move(go_to_get_ball));

    addStep(DribblerStep(robot_interface::DribblerMode::HIGH));

    DriveStep get_ball;
    get_ball.addFeature(BallTargetFeature(CircleShape("ball", cs.skills_config.robot_radius, false)));
    get_ball.setRotationControl(HeadingRotationControl("ball"));
    get_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    addStep(std::move(get_ball));

    DriveStep turn_to_target;
    turn_to_target.addFeature(TargetFeature(PointShape({TD_Pos::EXECUTING_ROBOT, 0}), 1000.0));
    turn_to_target.setRotationControl(HeadingRotationControl({TD_Pos::POINT, 0}));
    turn_to_target.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    addStep(std::move(turn_to_target));

    addStep(KickStep({TD, 0}, false, robot_interface::KickExecuteTime::WHEN_BALL_IN_DRIBBLER));

    DriveStep drive_in_ball;
    drive_in_ball.addFeature(BallTargetFeature(CircleShape("ball", 0.03, true), 1.0, true));
    drive_in_ball.setRotationControl(HeadingRotationControl({TD_Pos::POINT, 0}));
    drive_in_ball.setReachCondition(DriveStep::ReachCondition::NEVER);
    drive_in_ball.setCancelCondition(
        {CALLBACK, [&cs](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
             auto ball_pose = transform::Position(td.robot.getFrame()).getCurrentPosition(wm, "ball");
             return ball_pose.has_value() &&
                    ball_pose->translation().norm() > cs.skills_config.move_to_ball_pre_radius * 2;
         }});
    addStep(std::move(drive_in_ball));

    addStep(WaitStep(WAIT_DURATION, 0.2));
}
}  // namespace luhsoccer::skills