#include "skill_books/bod_skill_book/pass_ball_to_robot.hpp"
// include components here
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/steps/wait_step.hpp"
#include "local_planner_components/steps/kick_step.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/shapes/arc_shape.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/features/ball_target_feature.hpp"
#include "local_planner_components/features/robot_cf_obstacle.hpp"
#include "local_planner_components/steps/turn_around_ball_step.hpp"
#include "local_planner_components/steps/condition_step.hpp"
namespace luhsoccer::skills {

passBallToRobotBuild::passBallToRobotBuild()
    : SkillBuilder("passBallToRobot",  //
                   {"target_robot"},   //
                   {},                 //
                   {},                 //
                   {},                 //
                   {},                 //
                   {}){};

void passBallToRobotBuild::buildImpl(const config_provider::ConfigStore& cs) {
    ConditionStep cond({CALLBACK, [](const CallbackData& data) -> bool {
                            auto ball_data = data.wm->getBallInfo();
                            return ball_data.has_value() && ball_data->isInRobot(data.td.robot);
                        }});

    // ------------- Preposition -----------
    ComponentPosition target(TD_Pos::ROBOT, 0);
    ComponentPosition ball_rotated_to_target(
        CALLBACK,
        [target](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> transform::Position {
            std::optional<Eigen::Affine2d> target_pos = target.positionObject(wm, td).getCurrentPosition(wm, "ball");
            if (target_pos.has_value()) {
                double target_rot = atan2(target_pos->translation().y(), target_pos->translation().x());
                return {"ball", 0.0, 0.0, target_rot + L_PI};
            }
            return {"ball"};
        });

    DriveStep go_to_get_ball;
    go_to_get_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    go_to_get_ball.setAvoidOtherRobots(true);
    go_to_get_ball.setRotationControl(HeadingRotationControl("ball", false, L_PI / 2));
    go_to_get_ball.addFeature(TargetFeature(ArcShape(ball_rotated_to_target, cs.skills_config.move_to_ball_pre_radius,
                                                     cs.skills_config.kick_ball_through_target_preposition_angle),
                                            0.15));
    go_to_get_ball.addFeature(RobotCFObstacle(PointShape("ball"), cs.skills_config.ball_obstacle_weight));

    cond.addElseStep(DribblerStep(robot_interface::DribblerMode::OFF));
    cond.addElseStep(std::move(go_to_get_ball));
    cond.addElseStep(TurnAroundBallStep({TD_Pos::ROBOT, 0}));

    // ------- Direct --------

    DriveStep turn_to_target;
    turn_to_target.addFeature(TargetFeature(
        PointShape({CALLBACK,
                    [](const CallbackData& data) -> transform::Position {
                        auto start_pos = data.td.getCookie<transform::Position>(data.component_uid, "start_pos");
                        if (start_pos.has_value()) {
                            return start_pos.value();
                        }
                        auto transform = data.wm->getTransform(data.td.robot.getFrame());
                        if (transform.has_value()) {
                            start_pos = transform::Position("", transform->transform.translation().x(),
                                                            transform->transform.translation().y());
                            data.td.setCookie(data.component_uid, "start_pos", start_pos.value());
                            return start_pos.value();
                        }
                        return {""};
                    }}),
        1000.0));
    turn_to_target.setRotationControl(HeadingRotationControl({TD_Pos::ROBOT, 0}));
    turn_to_target.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);

    cond.addIfStep(DribblerStep(robot_interface::DribblerMode::HIGH));
    cond.addIfStep(std::move(turn_to_target));

    addStep(std::move(cond));

    DoubleComponentParam kick_strength{
        CALLBACK, [&cs](const CallbackData& data) -> double {
            auto trans = data.wm->getTransform(data.td.related_robots[0].getFrame(), data.td.robot.getFrame());
            if (!trans.has_value()) return 0.0;

            return cs.skills_config.kick_vel_offset +
                   trans->transform.translation().norm() * cs.skills_config.kick_vel_multiplier;
        }};
    addStep(KickStep(kick_strength, false, robot_interface::KickExecuteTime::WHEN_BALL_IN_DRIBBLER));

    DriveStep drive_in_ball;
    drive_in_ball.addFeature(BallTargetFeature(CircleShape("ball", 0.03, true), 1.0, true));
    drive_in_ball.setRotationControl(HeadingRotationControl({TD_Pos::ROBOT, 0}));
    drive_in_ball.setReachCondition(DriveStep::ReachCondition::NEVER);
    drive_in_ball.setCancelCondition(
        {CALLBACK, [&cs](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
             auto ball_pose = transform::Position(td.robot.getFrame()).getCurrentPosition(wm, "ball");
             return ball_pose.has_value() &&
                    ball_pose->translation().norm() > cs.skills_config.move_to_ball_pre_radius * 2;
         }});

    addStep(std::move(drive_in_ball));
    addStep(WaitStep(WAIT_DURATION, 0.2));

    // end of skill
}
}  // namespace luhsoccer::skills