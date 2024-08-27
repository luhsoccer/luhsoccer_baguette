#include "ball_steps.hpp"
#include "config/config_store.hpp"
#include "config/skills_config.hpp"
#include "robot_control/components/component_data.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/circle_shape.hpp"
#include "robot_control/components/shapes/line_shape.hpp"
#include "robot_control/components/steps/drive_step.hpp"

namespace luhsoccer::robot_control {
BoolComponentParam getBallMoving(const config_provider::ConfigStore& cs) {
    return {CALLBACK, [&cs](const ComponentData& comp_data, const ComponentUid&) -> bool {  //
                auto ball_vel = transform::Position("ball").getVelocity(comp_data.wm, "", "", comp_data.time);
                auto ball_info = comp_data.wm->getBallInfo(comp_data.time);
                bool ally_has_ball = false;
                bool ball_moves = false;

                if (ball_vel.has_value()) {
                    ball_moves = ball_vel->head<2>().norm() > cs.skills_config.ball_in_shot_vel;
                }
                if (ball_info.has_value()) {
                    if (ball_info->robot.has_value()) {
                        ally_has_ball = ball_info->robot->isAlly();
                    }
                }

                return (ball_moves && !ally_has_ball);
            }};
}

ComponentPosition getFutureBallPosition(const config_provider::ConfigStore& cs) {
    return {CALLBACK, [&cs](const ComponentData& comp_data, const ComponentUid&) -> transform::Position {
                transform::Position ball_pos("ball");
                auto past_ball_position =
                    ball_pos.getVelocity(comp_data.wm, "", "", comp_data.time, time::Duration(0.1));
                auto ball_pose = ball_pos.getCurrentPosition(comp_data.wm, "", comp_data.time);
                // try to stabilize the line shape by using the ball velocity and not the avg of the last
                // positions
                if (past_ball_position.has_value() && ball_pose.has_value() &&
                    past_ball_position->norm() > cs.skills_config.ball_in_shot_vel) {
                    transform::Position ball_vel("", ball_pose->translation().x() + past_ball_position->x(),
                                                 ball_pose->translation().y() + past_ball_position->y());
                    return ball_vel;
                }

                return comp_data.td.robot.getFrame();
            }};
}

std::shared_ptr<const AbstractStep> getWaitForShotStep(const config_provider::ConfigStore& cs) {
    DriveStep wait_for_kick;
    wait_for_kick.setRotationControl(HeadingRotationControl("ball"));
    wait_for_kick.setCancelCondition(getBallMoving(cs));
    wait_for_kick.setReachCondition(DriveStep::ReachCondition::NEVER);
    return std::make_shared<DriveStep>(std::move(wait_for_kick));
}

std::shared_ptr<const AbstractStep> getPrepositionForInterceptStep(const config_provider::ConfigStore& cs,
                                                                   const ComponentPosition& heading_target) {
    // drive to line of future ball position
    DriveStep receive_ball;
    receive_ball.setReachCondition(DriveStep::ReachCondition::NEVER);
    TargetFeature line_target(LineShape(getFutureBallPosition(cs), "ball", -LINE_CUTOFF_LENGTH, 0.0));
    line_target.setIgnoreVelocity(true);
    line_target.setKp(cs.skills_config.intercept_drive_k);
    line_target.setVelocityZeroForReach(false);
    line_target.setTranslationalTolerance(LINE_TRANS_TOLERANCE);
    receive_ball.addFeature(std::move(line_target));

    receive_ball.setRotationControl(HeadingRotationControl(heading_target));

    receive_ball.setCancelCondition(
        {CALLBACK, [&cs](const ComponentData& comp_data, const ComponentUid&) -> bool {
             auto ball_pos = transform::Position("ball").getCurrentPosition(comp_data.wm, "", comp_data.time);
             auto robot_pos = transform::Position(comp_data.td.robot.getFrame())
                                  .getCurrentPosition(comp_data.wm, "", comp_data.time);
             if (!ball_pos.has_value() || !robot_pos.has_value()) return false;
             auto distance_rob_ball = Eigen::Vector2d(robot_pos->translation().x() - ball_pos->translation().x(),
                                                      robot_pos->translation().y() - ball_pos->translation().y());

             auto field_size = comp_data.wm->getFieldData().size;

             // cancel if ball is out of field
             if (std::abs(ball_pos->translation().x()) > field_size.x() / 2 ||
                 std::abs(ball_pos->translation().y()) > field_size.y() / 2) {
                 return true;
             }

             // cancel if ball is 1.75m away from the robot and start with next drive step to absorb the ball
             if (robot_pos.has_value() && distance_rob_ball.norm() < cs.skills_config.intercept_get_ball_distance) {
                 return true;
             }

             return false;
         }});

    return std::make_shared<DriveStep>(std::move(receive_ball));
}

std::shared_ptr<DriveStep> getGetBallStep(const config_provider::ConfigStore& cs) {
    DriveStep get_ball;
    BallTargetFeature ball_target(CircleShape("ball", cs.skills_config.ball_get_ball_radius, false));
    ball_target.setKp(cs.skills_config.ball_get_ball_k_p_scaling);
    get_ball.addFeature(std::move(ball_target));
    get_ball.setRotationControl(HeadingRotationControl("ball"));
    get_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    return std::make_shared<DriveStep>(std::move(get_ball));
}
}  // namespace luhsoccer::robot_control