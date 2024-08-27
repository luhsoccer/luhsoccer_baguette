#include "skill_books/bod_skill_book/intercept_ball.hpp"
#include "local_planner_components/features/anti_target_feature.hpp"
#include "local_planner_components/features/ball_target_feature.hpp"
#include "local_planner_components/steps/condition_step.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/steps/wait_step.hpp"
#include "config/skills_config.hpp"

namespace luhsoccer::skills {

InterceptBallBuild::InterceptBallBuild()
    : SkillBuilder("InterceptBall",  //
                   {},               //
                   {},               //
                   {},               //
                   {},               //
                   {},               //
                   {}){};

void InterceptBallBuild::buildImpl(const config_provider::ConfigStore& cs) {
    addStep(DribblerStep(robot_interface::DribblerMode::LOW));

    // detect moving ball if not in ally robot
    BoolComponentParam ball_moving(
        CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> bool {
            auto ball_vel = wm->getVelocity("ball");
            auto ball_info = wm->getBallInfo();
            bool ally_has_ball = false;
            bool ball_moves = false;

            if (ball_vel.has_value()) {
                ball_moves = ball_vel->velocity.norm() > 0.5;
            }
            if (ball_info.has_value()) {
                if (ball_info->robot.has_value()) {
                    ally_has_ball = ball_info->robot->isAlly();
                }
            }

            return (ball_moves && !ally_has_ball);
        });

    // velocity extrapolation
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

            return td.robot.getFrame();
        });

    // Wait for the ball with the correct rotation
    DriveStep wait_for_kick;
    wait_for_kick.setRotationControl(HeadingRotationControl("ball"));
    wait_for_kick.setCancelCondition(ball_moving);
    wait_for_kick.setReachCondition(DriveStep::ReachCondition::NEVER);
    addStep(std::move(wait_for_kick));

    // // pass
    // DriveStep pass_ball_up;
    // pass_ball_up.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    // pass_ball_up.addFeature(TargetFeature(LineShape(
    //     past_ball_pos, "ball",
    //     {CALLBACK,
    //      [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
    //          // project onto field margins
    //          auto ball_pos = ComponentPosition("ball").positionObject(wm, td).getCurrentPosition(wm);
    //          auto past_ball_pos = wm->getVelocity("ball");

    //          // try to stabilize the line shape by using the ball velocity and not the avg of the last positions
    //          if (past_ball_pos.has_value() && ball_pos.has_value()) {
    //              double ball_vel = Eigen::Vector2d(past_ball_pos->velocity.x(), past_ball_pos->velocity.y()).norm();
    //              Eigen::Vector2d pose_in_sec(ball_pos->translation().x() + past_ball_pos->velocity.x(),
    //                                          ball_pos->translation().y() + past_ball_pos->velocity.y());
    //              if (-4.5 > pose_in_sec.x() || 4.5 < pose_in_sec.x() || -3.0 > pose_in_sec.y() ||
    //                  3.0 < pose_in_sec.y() ||
    //                  (pose_in_sec.y() < 1.0 && pose_in_sec.y() > -1.0 &&
    //                   (pose_in_sec.x() < -3.5 || pose_in_sec.x() > 3.5)) ||
    //                  (ball_pos->translation().y() < 1.0 && ball_pos->translation().y() > -1.0 &&
    //                   (ball_pos->translation().x() < -3.5 || ball_pos->translation().x() > 3.5))) {
    //                  return -10.0;
    //              }
    //              if (past_ball_pos->velocity.x() < 0) {
    //                  double orth_to_goalline_ally = ball_pos->translation().x() + 3.5;
    //                  double distance_to_goalline_ally =
    //                      ball_vel / abs(past_ball_pos->velocity.x()) * orth_to_goalline_ally;
    //                  double pos_on_y =
    //                      past_ball_pos->velocity.y() / past_ball_pos->velocity.x() * orth_to_goalline_ally +
    //                      ball_pos->translation().y();
    //                  if (pos_on_y < 1.0 && pos_on_y > -1.0) {
    //                      return -abs(distance_to_goalline_ally - ball_vel);
    //                  }

    //                  orth_to_goalline_ally = ball_pos->translation().x() + 4.5;
    //                  distance_to_goalline_ally = ball_vel / abs(past_ball_pos->velocity.x()) * orth_to_goalline_ally;
    //                  pos_on_y = past_ball_pos->velocity.y() / past_ball_pos->velocity.x() * orth_to_goalline_ally +
    //                             ball_pos->translation().y();
    //                  if (pos_on_y < 3.0 && pos_on_y > -3.0) {
    //                      return -abs(distance_to_goalline_ally - ball_vel);
    //                  }
    //              }
    //              if (past_ball_pos->velocity.x() > 0) {
    //                  double orth_to_goalline_ally = 3.5 - ball_pos->translation().x();
    //                  double distance_to_goalline_ally =
    //                      ball_vel / abs(past_ball_pos->velocity.x()) * orth_to_goalline_ally;
    //                  double pos_on_y =
    //                      past_ball_pos->velocity.y() / past_ball_pos->velocity.x() * orth_to_goalline_ally +
    //                      ball_pos->translation().y();
    //                  if (pos_on_y < 1.0 && pos_on_y > -1.0) {
    //                      return -abs(distance_to_goalline_ally - ball_vel);
    //                  }
    //                  orth_to_goalline_ally = 4.5 - ball_pos->translation().x();
    //                  distance_to_goalline_ally = ball_vel / abs(past_ball_pos->velocity.x()) * orth_to_goalline_ally;
    //                  pos_on_y = past_ball_pos->velocity.y() / past_ball_pos->velocity.x() * orth_to_goalline_ally +
    //                             ball_pos->translation().y();
    //                  if (pos_on_y < 3.0 && pos_on_y > -3.0) {
    //                      return -abs(distance_to_goalline_ally - ball_vel);
    //                  }
    //              }
    //              if (past_ball_pos->velocity.y() < 0) {
    //                  double orth_to_goalline_ally = ball_pos->translation().y() + 3.0;
    //                  double distance_to_goalline_ally =
    //                      ball_vel / abs(past_ball_pos->velocity.y()) * orth_to_goalline_ally;
    //                  double pos_on_x =
    //                      past_ball_pos->velocity.x() / past_ball_pos->velocity.y() * orth_to_goalline_ally +
    //                      ball_pos->translation().x();
    //                  if (pos_on_x < 4.5 && pos_on_x > -4.5) {
    //                      return -abs(distance_to_goalline_ally - ball_vel);
    //                  }
    //              }
    //              if (past_ball_pos->velocity.y() > 0) {
    //                  double orth_to_goalline_ally = 3.0 - ball_pos->translation().y();
    //                  double distance_to_goalline_ally =
    //                      ball_vel / abs(past_ball_pos->velocity.y()) * orth_to_goalline_ally;
    //                  double pos_on_x =
    //                      past_ball_pos->velocity.x() / past_ball_pos->velocity.y() * orth_to_goalline_ally +
    //                      ball_pos->translation().x();
    //                  if (pos_on_x < 4.5 && pos_on_x > -4.5) {
    //                      return -abs(distance_to_goalline_ally - ball_vel);
    //                  }
    //              }
    //          }
    //          return -10.0;
    //      }},
    //     {CALLBACK, [&cs](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
    //          // project onto field margins
    //          auto ball_pos = ComponentPosition("ball").positionObject(wm, td).getCurrentPosition(wm);
    //          auto past_ball_pos = wm->getVelocity("ball");

    //          // try to stabilize the line shape by using the ball velocity and not the avg of the last positions
    //          if (past_ball_pos.has_value() && ball_pos.has_value()) {
    //              double ball_vel = Eigen::Vector2d(past_ball_pos->velocity.x(), past_ball_pos->velocity.y()).norm();
    //              Eigen::Vector2d pose_in_sec(ball_pos->translation().x() + past_ball_pos->velocity.x(),
    //                                          ball_pos->translation().y() + past_ball_pos->velocity.y());
    //              if (-4.5 > pose_in_sec.x() || 4.5 < pose_in_sec.x() || -3.0 > pose_in_sec.y() ||
    //                  3.0 < pose_in_sec.y() ||
    //                  (pose_in_sec.y() < 1.0 && pose_in_sec.y() > -1.0 &&
    //                   (pose_in_sec.x() < -3.5 || pose_in_sec.x() > 3.5)) ||
    //                  (ball_pos->translation().y() < 1.0 && ball_pos->translation().y() > -1.0 &&
    //                   (ball_pos->translation().x() < -3.5 || ball_pos->translation().x() > 3.5))) {
    //                  return -10.0;
    //              }
    //          }
    //          return cs.skills_config.move_to_ball_pre_radius;
    //      }})));
    // pass_ball_up.addFeature(
    //     TargetFeature(LineShape(past_ball_pos, "ball", 0.0, 0.0), DEFAULT_TRANSLATIONAL_TOLERANCE, 0.001));
    // pass_ball_up.setRotationControl(HeadingRotationControl("ball"));
    // pass_ball_up.addFeature(RobotCFObstacle(PointShape("ball"), cs.skills_config.ball_obstacle_weight));
    // pass_ball_up.setCancelCondition(
    //     {CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> bool {
    //          auto ball_pos = ComponentPosition("ball").positionObject(wm, td).getCurrentPosition(wm);

    //          if (ball_pos.has_value()) {
    //              if (ball_pos->translation().x() > 4.5 || ball_pos->translation().x() < -4.5 ||
    //                  ball_pos->translation().y() > 3.0 || ball_pos->translation().y() < -3.0) {
    //                  return true;
    //              }
    //          }

    //          auto ball_vel = wm->getVelocity("ball");
    //          if (ball_vel.has_value() && ball_vel->velocity.norm() < 0.5) return true;

    //          auto ball_info = wm->getBallInfo();

    //          if (ball_info.has_value()) {
    //              auto robot_info = ball_info->robot;
    //              if (robot_info.has_value()) {
    //                  if (robot_info->getFrame() == td.robot.getFrame()) {
    //                      return true;
    //                  }
    //              }
    //          }

    //          return false;
    //      }});

    // addStep(std::move(pass_ball_up));

    // drive to line of future ball position
    DriveStep receive_ball;
    receive_ball.setReachCondition(DriveStep::ReachCondition::ALL_TARGETS);
    receive_ball.addFeature(TargetFeature(LineShape(past_ball_pos, "ball", -10.0, -10.0), 0.01, 1.0, false, false,
                                          cs.skills_config.receive_ball_kg, cs.skills_config.receive_ball_kv));
    receive_ball.setRotationControl(HeadingRotationControl("ball"));
    receive_ball.setCancelCondition(
        {CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> bool {
             auto ball_pos = ComponentPosition("ball").positionObject(wm, td).getCurrentPosition(wm);
             auto robot_pos = ComponentPosition(td.robot.getFrame()).positionObject(wm, td).getCurrentPosition(wm);
             auto distance_rob_ball = Eigen::Vector2d(robot_pos->translation().x() - ball_pos->translation().x(),
                                                      robot_pos->translation().y() - ball_pos->translation().y());

             if (ball_pos.has_value()) {
                 auto field_size = wm->getFieldData().size;
                 auto field_height = field_size.y();
                 auto field_width = field_size.x();

                 // cancel if ball is out of field
                 if (ball_pos->translation().x() > field_width / 2 ||
                     ball_pos->translation().x() < -(field_width / 2) ||
                     ball_pos->translation().y() > field_height / 2 ||
                     ball_pos->translation().y() < -(field_height / 2)) {
                     return true;
                 }

                 // cancel if ball is 1.75m away from the robot and start with next drive step to absorb the ball
                 if (robot_pos.has_value() && distance_rob_ball.norm() < 1.5) {
                     return true;
                 }
             }

             return false;
         }});

    addStep(std::move(receive_ball));
    addStep(DribblerStep(robot_interface::DribblerMode::HIGH));

    // BoolComponentParam ball_slow_enough(
    //     CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> bool {
    //         auto ball_vel = wm->getVelocity("ball");

    //         if (ball_vel.has_value() && ball_vel->velocity.norm() < 0.1) {
    //             return true;
    //         }

    //         return false;
    //     });

    BoolComponentParam conditions(
        CALLBACK, [&cs](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> bool {
            auto ball_info = wm->getBallInfo();
            auto robot_pos = ComponentPosition(td.robot.getFrame()).positionObject(wm, td).getCurrentPosition(wm);

            // cancel if ball is in dribbler
            if (ball_info.has_value() && ball_info->isInRobot(td.robot)) return true;

            if (robot_pos.has_value()) {
                auto field_size = wm->getFieldData().size;
                auto field_height = field_size.y();
                auto field_width = field_size.x();

                // cancel if robot is outside of the field + margin
                double margin = cs.skills_config.receive_ball_margin;
                if (robot_pos->translation().x() > field_width / 2 - margin ||
                    robot_pos->translation().x() < -(field_width / 2 - margin) ||
                    robot_pos->translation().y() > field_height / 2 - margin ||
                    robot_pos->translation().y() < -(field_height / 2 - margin)) {
                    return true;
                }
            }

            return false;
        });

    DoubleComponentParam ball_speed_x(
        CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
            auto ball_vel = wm->getVelocity("ball");
            if (ball_vel.has_value()) {
                return ball_vel->velocity.x() * 1.5;
            }

            return 1.0;
        });
    DoubleComponentParam ball_speed_y(
        CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
            auto ball_vel = wm->getVelocity("ball");
            if (ball_vel.has_value()) {
                return ball_vel->velocity.y() * 1.5;
            }

            return 1.0;
        });

    DoubleComponentParam anti_target_weight(
        CALLBACK, [&cs](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            auto robot_pos = ComponentPosition(td.robot.getFrame()).positionObject(wm, td).getCurrentPosition(wm);

            // cancel if robot is outside of the field including margin
            if (robot_pos.has_value()) {
                auto field_size = wm->getFieldData().size;
                auto field_height = field_size.y();
                auto field_width = field_size.x();

                // cancel if robot is outside of the field + margin
                double margin = cs.skills_config.receive_ball_margin;
                if (robot_pos->translation().x() > field_width / 2 - margin ||
                    robot_pos->translation().x() < -(field_width / 2 - margin) ||
                    robot_pos->translation().y() > field_height / 2 - margin ||
                    robot_pos->translation().y() < -(field_height / 2 - margin)) {
                    return 0.0;
                }

                // cancel if robot is goalie
                auto goalie_id = wm->getGoalieID();
                if (goalie_id.has_value() && *goalie_id == td.robot) {
                    return 0.0;
                }
            }

            return 1.0;
        });

    DriveStep absorb;
    absorb.setRotationControl(HeadingRotationControl("ball"));
    absorb.setReachCondition(DriveStep::ReachCondition::NEVER);
    absorb.addFeature(TargetFeature(LineShape(past_ball_pos, "ball", -10.0, -10.0), 0.01, 1.0, false, false,
                                    cs.skills_config.receive_ball_kg, cs.skills_config.receive_ball_kv));
    absorb.addFeature(AntiTargetFeature(PointShape("ball"), 1.5, anti_target_weight));
    absorb.setCancelCondition(conditions);
    absorb.setMaxVelX(ball_speed_x);
    absorb.setMaxVelY(ball_speed_y);

    addStep(std::move(absorb));
}
}  // namespace luhsoccer::skills