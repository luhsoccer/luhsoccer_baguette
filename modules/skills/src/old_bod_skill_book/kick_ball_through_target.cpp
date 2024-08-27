#include "skill_books/bod_skill_book/kick_ball_through_target.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
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
#include "config/skills_config.hpp"

namespace luhsoccer::skills {

KickBallThroughTargetBuild::KickBallThroughTargetBuild()
    : SkillBuilder("KickBallThroughTarget",  //
                   {},                       //
                   {"TargetPoint"},          //
                   {"KickVelocity"},         //
                   {},                       //
                   {},                       //
                   {}){};

void KickBallThroughTargetBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // PARAMS: ball velocity FUTURE ball position
    ComponentPosition past_ball_pos(
        CALLBACK,
        [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> transform::Position {
            ComponentPosition ball_pos("ball");
            auto past_ball_pos = ball_pos.positionObject(wm, td).getVelocity(wm);

            // try to stabilize the line shape by using the ball velocity and not the avg of the last positions
            if (past_ball_pos.has_value() && past_ball_pos->norm() > 0.5) {
                transform::Position ball_vel(
                    "", ball_pos.positionObject(wm, td).getCurrentPosition(wm)->translation().x() + past_ball_pos->x(),
                    ball_pos.positionObject(wm, td).getCurrentPosition(wm)->translation().y() + past_ball_pos->y());
                return ball_vel;
            }

            return ComponentPosition(td.robot.getFrame()).positionObject(wm, td);
        });

    // Case 2: We have the ball already in our dribbler
    ConditionStep cond({CALLBACK, [](const CallbackData& data) -> bool {
                            auto ball_data = data.wm->getBallInfo();
                            return ball_data.has_value() && ball_data->isInRobot(data.td.robot);
                        }});
    // Case 3: we catch up with ball rolling towards us
    ConditionStep ball_to_fast(
        {CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> bool {
             auto ball_vel = wm->getVelocity("ball");
             auto robot_pos = ComponentPosition(td.robot.getFrame()).positionObject(wm, td).getCurrentPosition(wm);
             auto ball_pos = ComponentPosition("ball").positionObject(wm, td).getCurrentPosition(wm);
             auto ball_info = wm->getBallInfo();

             if (ball_info.has_value()) {
                 if (ball_info->robot.has_value() && ball_info->robot->isAlly()) {
                     return false;
                 }
             }

             if (ball_vel.has_value() && robot_pos.has_value() && ball_pos.has_value() &&
                 ball_vel->velocity.norm() > 0.6) {
                 double distance_rob_ball = Eigen::Vector2d(robot_pos->translation().x() - ball_pos->translation().x(),
                                                            robot_pos->translation().y() - ball_pos->translation().y())
                                                .norm();
                 double distance_rob_ball_vel = Eigen::Vector2d(robot_pos->translation().x() - ball_vel->velocity.x(),
                                                                robot_pos->translation().y() - ball_vel->velocity.y())
                                                    .norm();

                 if (distance_rob_ball > distance_rob_ball_vel) {
                     return true;
                 }
             }

             return false;
         }});

    // ------------- intercept ball -------------

    ball_to_fast.addIfStep(DribblerStep(robot_interface::DribblerMode::LOW));

    /*DriveStep pass_ball_up;
    pass_ball_up.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    pass_ball_up.addFeature(TargetFeature(LineShape(
        past_ball_pos, "ball",
        {CALLBACK,
         [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
             // project onto field margins
             auto ball_pos = ComponentPosition("ball").positionObject(wm, td).getCurrentPosition(wm);
             auto past_ball_pos = wm->getVelocity("ball");

             // try to stabilize the line shape by using the ball velocity and not the avg of the last positions
             if (past_ball_pos.has_value() && ball_pos.has_value()) {
                 double ball_vel = Eigen::Vector2d(past_ball_pos->velocity.x(), past_ball_pos->velocity.y()).norm();
                 Eigen::Vector2d pose_in_sec(ball_pos->translation().x() + past_ball_pos->velocity.x(),
                                             ball_pos->translation().y() + past_ball_pos->velocity.y());
                 if (-4.5 > pose_in_sec.x() || 4.5 < pose_in_sec.x() || -3.0 > pose_in_sec.y() ||
                     3.0 < pose_in_sec.y() ||
                     (pose_in_sec.y() < 1.0 && pose_in_sec.y() > -1.0 &&
                      (pose_in_sec.x() < -3.5 || pose_in_sec.x() > 3.5)) ||
                     (ball_pos->translation().y() < 1.0 && ball_pos->translation().y() > -1.0 &&
                      (ball_pos->translation().x() < -3.5 || ball_pos->translation().x() > 3.5))) {
                     return -10.0;
                 }
                 if (past_ball_pos->velocity.x() < 0) {
                     double orth_to_goalline_ally = ball_pos->translation().x() + 3.5;
                     double distance_to_goalline_ally =
                         ball_vel / abs(past_ball_pos->velocity.x()) * orth_to_goalline_ally;
                     double pos_on_y =
                         past_ball_pos->velocity.y() / past_ball_pos->velocity.x() * orth_to_goalline_ally +
                         ball_pos->translation().y();
                     if (pos_on_y < 1.0 && pos_on_y > -1.0) {
                         return -abs(distance_to_goalline_ally - ball_vel);
                     }

                     orth_to_goalline_ally = ball_pos->translation().x() + 4.5;
                     distance_to_goalline_ally = ball_vel / abs(past_ball_pos->velocity.x()) * orth_to_goalline_ally;
                     pos_on_y = past_ball_pos->velocity.y() / past_ball_pos->velocity.x() * orth_to_goalline_ally +
                                ball_pos->translation().y();
                     if (pos_on_y < 3.0 && pos_on_y > -3.0) {
                         return -abs(distance_to_goalline_ally - ball_vel);
                     }
                 }
                 if (past_ball_pos->velocity.x() > 0) {
                     double orth_to_goalline_ally = 3.5 - ball_pos->translation().x();
                     double distance_to_goalline_ally =
                         ball_vel / abs(past_ball_pos->velocity.x()) * orth_to_goalline_ally;
                     double pos_on_y =
                         past_ball_pos->velocity.y() / past_ball_pos->velocity.x() * orth_to_goalline_ally +
                         ball_pos->translation().y();
                     if (pos_on_y < 1.0 && pos_on_y > -1.0) {
                         return -abs(distance_to_goalline_ally - ball_vel);
                     }
                     orth_to_goalline_ally = 4.5 - ball_pos->translation().x();
                     distance_to_goalline_ally = ball_vel / abs(past_ball_pos->velocity.x()) * orth_to_goalline_ally;
                     pos_on_y = past_ball_pos->velocity.y() / past_ball_pos->velocity.x() * orth_to_goalline_ally +
                                ball_pos->translation().y();
                     if (pos_on_y < 3.0 && pos_on_y > -3.0) {
                         return -abs(distance_to_goalline_ally - ball_vel);
                     }
                 }
                 if (past_ball_pos->velocity.y() < 0) {
                     double orth_to_goalline_ally = ball_pos->translation().y() + 3.0;
                     double distance_to_goalline_ally =
                         ball_vel / abs(past_ball_pos->velocity.y()) * orth_to_goalline_ally;
                     double pos_on_x =
                         past_ball_pos->velocity.x() / past_ball_pos->velocity.y() * orth_to_goalline_ally +
                         ball_pos->translation().x();
                     if (pos_on_x < 4.5 && pos_on_x > -4.5) {
                         return -abs(distance_to_goalline_ally - ball_vel);
                     }
                 }
                 if (past_ball_pos->velocity.y() > 0) {
                     double orth_to_goalline_ally = 3.0 - ball_pos->translation().y();
                     double distance_to_goalline_ally =
                         ball_vel / abs(past_ball_pos->velocity.y()) * orth_to_goalline_ally;
                     double pos_on_x =
                         past_ball_pos->velocity.x() / past_ball_pos->velocity.y() * orth_to_goalline_ally +
                         ball_pos->translation().x();
                     if (pos_on_x < 4.5 && pos_on_x > -4.5) {
                         return -abs(distance_to_goalline_ally - ball_vel);
                     }
                 }
             }
             return -10.0;
         }},
        {CALLBACK, [&cs](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
             // project onto field margins
             auto ball_pos = ComponentPosition("ball").positionObject(wm, td).getCurrentPosition(wm);
             auto past_ball_pos = wm->getVelocity("ball");

             // try to stabilize the line shape by using the ball velocity and not the avg of the last positions
             if (past_ball_pos.has_value() && ball_pos.has_value()) {
                 Eigen::Vector2d pose_in_sec(ball_pos->translation().x() + past_ball_pos->velocity.x(),
                                             ball_pos->translation().y() + past_ball_pos->velocity.y());
                 if (-4.5 > pose_in_sec.x() || 4.5 < pose_in_sec.x() || -3.0 > pose_in_sec.y() ||
                     3.0 < pose_in_sec.y() ||
                     (pose_in_sec.y() < 1.0 && pose_in_sec.y() > -1.0 &&
                      (pose_in_sec.x() < -3.5 || pose_in_sec.x() > 3.5)) ||
                     (ball_pos->translation().y() < 1.0 && ball_pos->translation().y() > -1.0 &&
                      (ball_pos->translation().x() < -3.5 || ball_pos->translation().x() > 3.5))) {
                     return -10.0;
                 }
             }
             return cs.skills_config.move_to_ball_pre_radius;
         }})));
    pass_ball_up.addFeature(
        TargetFeature(LineShape(past_ball_pos, "ball", 0.0, 0.0), DEFAULT_TRANSLATIONAL_TOLERANCE, 0.001));
    pass_ball_up.setRotationControl(HeadingRotationControl("ball"));
    pass_ball_up.addFeature(RobotCFObstacle(PointShape("ball"), cs.skills_config.ball_obstacle_weight));
    pass_ball_up.setCancelCondition(
        {CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> bool {
             auto ball_pos = ComponentPosition("ball").positionObject(wm, td).getCurrentPosition(wm);

             if (ball_pos.has_value()) {
                 auto field_size = wm->getFieldData().size;
                 auto field_height = field_size.y();
                 auto field_width = field_size.x();

                 if (ball_pos->translation().x() > field_width / 2 ||
                     ball_pos->translation().x() < -(field_width / 2) ||
                     ball_pos->translation().y() > field_height / 2 ||
                     ball_pos->translation().y() < -(field_height / 2)) {
                     return true;
                 }
             }

             auto ball_vel = wm->getVelocity("ball");
             if (ball_vel.has_value() && ball_vel->velocity.norm() < 0.5) return true;

             return false;
         }});*/

    // DriveStep for future ball position
    DriveStep receive_ball;
    receive_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);

    receive_ball.addFeature(BallTargetFeature(LineShape(past_ball_pos, "ball", -10, -0.2), 1.0, true,
                                              cs.skills_config.receive_ball_kg, cs.skills_config.receive_ball_kv));
    receive_ball.setRotationControl(HeadingRotationControl("ball"));
    receive_ball.setCancelCondition(
        {CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> bool {
             auto ball_pos = ComponentPosition("ball").positionObject(wm, td).getCurrentPosition(wm);

             if (ball_pos.has_value()) {
                 auto field_size = wm->getFieldData().size;
                 auto field_height = field_size.y();
                 auto field_width = field_size.x();

                 if (ball_pos->translation().x() > field_width / 2 ||
                     ball_pos->translation().x() < -(field_width / 2) ||
                     ball_pos->translation().y() > field_height / 2 ||
                     ball_pos->translation().y() < -(field_height / 2)) {
                     return true;
                 }
             }

             auto ball_vel = wm->getVelocity("ball");
             if (ball_vel.has_value() && ball_vel->velocity.norm() < 0.5) return true;
             return false;
         }});

    // ball_to_fast.addIfStep(std::move(pass_ball_up));
    ball_to_fast.addIfStep(std::move(receive_ball));
    ball_to_fast.addIfStep(DribblerStep(robot_interface::DribblerMode::HIGH));
    addStep(std::move(ball_to_fast));

    // ------------- Preposition -----------
    ComponentPosition target(TD_Pos::POINT, 0);
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
                                            0.1));
    go_to_get_ball.addFeature(RobotCFObstacle(PointShape("ball"), cs.skills_config.ball_obstacle_weight));

    cond.addElseStep(DribblerStep(robot_interface::DribblerMode::OFF));
    ConditionStep move_to_ball_decision({CALLBACK, [&cs](const CallbackData& data) {
                                             auto pos = data.wm->getTransform("ball", data.td.robot.getFrame());
                                             if (!pos.has_value()) return false;
                                             return pos->transform.translation().norm() >
                                                    cs.skills_config.move_to_ball_pre_radius;
                                         }});
    move_to_ball_decision.addIfStep(std::move(go_to_get_ball));
    cond.addElseStep(std::move(move_to_ball_decision));
    cond.addElseStep(TurnAroundBallStep({TD_Pos::POINT, 0}));

    // ------- Direct --------
    DriveStep turn_to_target;
    turn_to_target.addFeature(TargetFeature(
        CircleShape({CALLBACK,
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
                     }},
                    0.1, true),
        1000.0));
    turn_to_target.setRotationControl(
        HeadingRotationControl({TD_Pos::POINT, 0}, false, THREE_DEGREE_IN_RADIAN / 3 * 2, false));
    turn_to_target.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);

    cond.addIfStep(DribblerStep(robot_interface::DribblerMode::LOW));
    cond.addIfStep(std::move(turn_to_target));

    addStep(std::move(cond));

    addStep(KickStep({TD, 0}, false, robot_interface::KickExecuteTime::WHEN_BALL_IN_DRIBBLER));
    addStep(WaitStep(WAIT_DURATION, 0.1));
    DriveStep drive_in_ball;
    drive_in_ball.addFeature(BallTargetFeature(CircleShape("ball", 0.03, true), 0.1, true));
    drive_in_ball.addFeature(TargetFeature(LineShape({"ball"}, {TD_Pos::POINT, 0}, -1.0), 0.05, 1.0, false));
    drive_in_ball.setMaxVelX(0.5);
    drive_in_ball.setMaxVelY(0.5);
    drive_in_ball.setAvoidOtherRobots(false);
    drive_in_ball.setRotationControl(HeadingRotationControl({TD_Pos::POINT, 0}));
    drive_in_ball.setReachCondition(DriveStep::ReachCondition::NEVER);
    drive_in_ball.setCancelCondition(
        {CALLBACK, [&cs](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
             auto ball_pose = transform::Position("ball").getCurrentPosition(wm, td.robot.getFrame());
             if (!ball_pose.has_value()) return false;
             bool ball_shot = ball_pose->translation().norm() > cs.skills_config.move_to_ball_pre_radius * 2;
             bool ball_beside_robot =
                 std::abs(ball_pose->translation().y()) > 0.06 && ball_pose->translation().x() < 0.08;
             return ball_shot || ball_beside_robot;
         }});

    addStep(std::move(drive_in_ball));
    addStep(WaitStep(WAIT_DURATION, 0.2));

    // end of skill
}
}  // namespace luhsoccer::skills