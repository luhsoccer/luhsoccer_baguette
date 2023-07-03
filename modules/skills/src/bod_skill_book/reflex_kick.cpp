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
    // Add skill definition here. Use addStep to add a step
    BoolComponentParam ball_moving(
        CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> bool {
            ComponentPosition ball_pos("ball");
            time::TimePoint past_time = time::now() - time::Duration(0.2);

            auto past_ball_pos = ball_pos.positionObject(wm, td).getCurrentPosition(wm, "", past_time);
            auto current_ball_pos = ball_pos.positionObject(wm, td).getCurrentPosition(wm, "", time::now());

            if (!past_ball_pos.has_value() || !current_ball_pos.has_value()) return true;

            auto ball_owner = wm->getBallInfo()->robot;
            bool ally_has_ball = ball_owner.has_value() && ball_owner->isAlly();

            return (current_ball_pos->translation() - past_ball_pos->translation()).norm() / 0.2 > 1.0 &&
                   !ally_has_ball;
        });

    DriveStep wait_for_kick;
    wait_for_kick.setRotationControl(HeadingRotationControl({TD_Pos::POINT, 0}));
    wait_for_kick.setCancelCondition(ball_moving);
    wait_for_kick.setReachCondition(DriveStep::ReachCondition::NEVER);
    addStep(std::move(wait_for_kick));

    addStep(KickStep({TD, 0}, false, robot_interface::KickExecuteTime::WHEN_BALL_IN_DRIBBLER));

    DriveStep receive_ball;

    receive_ball.setReachCondition(DriveStep::ReachCondition::ALL_TARGETS);
    receive_ball.setRotationControl(HeadingRotationControl({TD_Pos::POINT, 0}));

    // Ball LineShape
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

    receive_ball.addFeature(
        BallTargetFeature(LineShape(past_ball_pos, ball_pos_offset, -10.0, -10.0), 1.0, true, 0.15));

    addStep(std::move(receive_ball));

    // end of skill
}
}  // namespace luhsoccer::skills