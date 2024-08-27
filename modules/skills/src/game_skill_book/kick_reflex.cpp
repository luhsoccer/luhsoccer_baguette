#include "skill_books/game_skill_book/kick_reflex.hpp"
#include "ball_steps.hpp"
#include "config/skills_config.hpp"
#include "robot_control/components/component_data.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/line_shape.hpp"
#include "robot_control/components/steps/dribbler_step.hpp"
#include "robot_control/components/steps/drive_step.hpp"
#include "robot_control/components/steps/kick_step.hpp"
// include components here

namespace luhsoccer::skills {

KickReflexBuild::KickReflexBuild()
    : SkillBuilder("KickReflex",       //
                   {},                 //
                   {"Target point"},   //
                   {"Kick velocity"},  //
                   {},                 //
                   {},                 //
                   {}){};

void KickReflexBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    addStep(DribblerStep(robot_interface::DribblerMode::OFF));

    addStepPtr(getWaitForShotStep(cs));

    addStep(KickStep({TD, 0}, false, robot_interface::KickExecuteTime::WHEN_BALL_IN_DRIBBLER));

    DriveStep receive_ball;

    receive_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    receive_ball.setRotationControl(HeadingRotationControl({TD_Pos::POINT, 0}));

    // improvise a LineShape using the past ball position
    auto future_ball_pos = getFutureBallPosition(cs);
    ComponentPosition future_ball_pos_offset(
        CALLBACK, [&cs, future_ball_pos](const ComponentData& comp_data, const ComponentUid&) -> transform::Position {
            auto future_ball_position =
                future_ball_pos.positionObject(comp_data).getCurrentPosition(comp_data.wm, "", comp_data.time);
            auto robot_pose =
                transform::Position(comp_data.td.robot.getFrame()).getCurrentPosition(comp_data.wm, "", comp_data.time);

            if (!future_ball_position.has_value() || !robot_pose.has_value()) return {""};
            // try to stabilize the line shape by using the ball velocity and not the avg of the last positions

            Eigen::Rotation2Dd robot_angle(robot_pose->rotation());

            return {"",
                    (future_ball_position->translation().x() +
                     (std::cos(robot_angle.angle())) * -cs.skills_config.reflex_kick_offset),
                    future_ball_position->translation().y() +
                        (std::sin(robot_angle.angle()) * -cs.skills_config.reflex_kick_offset)};
        });

    ComponentPosition ball_pos_offset(
        {CALLBACK, [&cs](const ComponentData& comp_data, const ComponentUid&) -> transform::Position {
             ;

             auto robot_pos_affine = transform::Position(comp_data.td.robot.getFrame())
                                         .getCurrentPosition(comp_data.wm, "", comp_data.time);

             if (!robot_pos_affine.has_value()) return {"ball"};

             Eigen::Rotation2Dd robot_angle(robot_pos_affine->rotation());

             return {"ball", ((std::cos(robot_angle.angle())) * -cs.skills_config.reflex_kick_offset),
                     (std::sin(robot_angle.angle()) * -cs.skills_config.reflex_kick_offset)};
         }});

    BallTargetFeature ball_target(LineShape(future_ball_pos_offset, ball_pos_offset, -LINE_CUTOFF_LENGTH, 0.0));
    ball_target.setIgnoreVelocity(true);
    receive_ball.addFeature(std::move(ball_target));

    BoolComponentParam ball_moving_away(CALLBACK, [&cs](const ComponentData& comp_data, const ComponentUid&) {
        auto ball_vel = transform::Position("ball").getVelocity(comp_data.wm, "", "", comp_data.time);
        auto robot_pos_in_ball =
            transform::Position(comp_data.td.robot.getFrame()).getCurrentPosition(comp_data.wm, "ball", comp_data.time);

        if (!ball_vel.has_value() || !robot_pos_in_ball.has_value()) return false;
        if (ball_vel->head<2>().norm() < cs.skills_config.ball_in_shot_vel) return false;

        return ball_vel->head<2>().dot(robot_pos_in_ball->translation()) < 0;
    });
    receive_ball.setCancelCondition(ball_moving_away);

    addStep(std::move(receive_ball));

    // end of skill
}
}  // namespace luhsoccer::skills