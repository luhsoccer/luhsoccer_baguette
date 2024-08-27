#include "skill_books/bod_skill_book/reflex_kick.hpp"
// include components here

#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/features/ball_target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/point_shape.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/steps/kick_step.hpp"
#include "config/skills_config.hpp"

namespace luhsoccer::skills {

ReflexKickBuild::ReflexKickBuild()
    : SkillBuilder("ReflexKick",        //
                   {},                  //
                   {"TargetPosition"},  //
                   {"kick_velocity"},   //
                   {},                  //
                   {},                  //
                   {}){};

void ReflexKickBuild::buildImpl(const config_provider::ConfigStore& cs) {
    BoolComponentParam ball_moving(
        CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> bool {
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

    DriveStep wait_for_kick;
    wait_for_kick.setRotationControl(HeadingRotationControl({TD_Pos::POINT, 0}));
    wait_for_kick.setCancelCondition(ball_moving);
    wait_for_kick.setReachCondition(DriveStep::ReachCondition::NEVER);
    addStep(std::move(wait_for_kick));

    addStep(KickStep({TD, 0}, false, robot_interface::KickExecuteTime::WHEN_BALL_IN_DRIBBLER));

    DriveStep receive_ball;

    receive_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    receive_ball.setRotationControl(HeadingRotationControl({TD_Pos::POINT, 0}));

    // improvise a LineShape using the past ball position
    ComponentPosition past_ball_pos(
        CALLBACK,
        [&cs](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> transform::Position {
            ComponentPosition ball_pos("ball");
            auto past_ball_pos = ball_pos.positionObject(wm, td).getVelocity(wm);
            ComponentPosition robot_pos(td.robot.getFrame());
            auto robot_pos_affine = robot_pos.positionObject(wm, td).getCurrentPosition(wm);

            if (past_ball_pos.has_value() && robot_pos_affine.has_value()) {
                // try to stabilize the line shape by using the ball velocity and not the avg of the last positions
                if (past_ball_pos->norm() > 0.5) {
                    transform::Position ball_vel(
                        "",
                        ball_pos.positionObject(wm, td).getCurrentPosition(wm)->translation().x() - past_ball_pos->x(),
                        ball_pos.positionObject(wm, td).getCurrentPosition(wm)->translation().y() - past_ball_pos->y());

                    Eigen::Rotation2Dd robot_angle(robot_pos_affine->rotation());

                    return {"",
                            (ball_vel.getCurrentPosition(wm)->translation().x() +
                             (std::cos(robot_angle.angle())) * -cs.skills_config.reflex_kick_dribbler_offset),
                            ball_vel.getCurrentPosition(wm)->translation().y() +
                                (std::sin(robot_angle.angle()) * -cs.skills_config.reflex_kick_dribbler_offset)};
                }

                return robot_pos.positionObject(wm, td);
            }
            return {"", 0.0, 0.0};
        });

    ComponentPosition ball_pos_offset(
        {CALLBACK,
         [&cs](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> transform::Position {
             ComponentPosition ball_pos("ball");
             ComponentPosition robot_pos(td.robot.getFrame());

             auto robot_pos_affine = robot_pos.positionObject(wm, td).getCurrentPosition(wm);
             auto ball_pos_affine = ball_pos.positionObject(wm, td).getCurrentPosition(wm);

             if (!robot_pos_affine.has_value() || !ball_pos_affine.has_value()) return {""};

             Eigen::Rotation2Dd robot_angle(robot_pos_affine->rotation());

             return {"",
                     (ball_pos_affine->translation().x() +
                      (std::cos(robot_angle.angle())) * -cs.skills_config.reflex_kick_dribbler_offset),
                     ball_pos_affine->translation().y() +
                         (std::sin(robot_angle.angle()) * -cs.skills_config.reflex_kick_dribbler_offset)};
         }});

    receive_ball.addFeature(BallTargetFeature(LineShape(past_ball_pos, ball_pos_offset, -10.0, -10.0), 1.0, true,
                                              cs.skills_config.receive_ball_kg, cs.skills_config.receive_ball_kv));

    addStep(std::move(receive_ball));
}
}  // namespace luhsoccer::skills