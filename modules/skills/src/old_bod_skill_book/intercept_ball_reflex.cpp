#include "skill_books/bod_skill_book/intercept_ball_reflex.hpp"
#include "local_planner_components/features/ball_target_feature.hpp"
#include "local_planner_components/steps/condition_step.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/steps/kick_step.hpp"
#include "local_planner_components/steps/wait_step.hpp"
#include "config/skills_config.hpp"

namespace luhsoccer::skills {

InterceptBallReflexBuild::InterceptBallReflexBuild()
    : SkillBuilder("InterceptBallReflex",  //
                   {},                     //
                   {},                     //
                   {},                     //
                   {},                     //
                   {},                     //
                   {}){};

void InterceptBallReflexBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // turn on dribbler
    addStep(DribblerStep(robot_interface::DribblerMode::LOW));

    // detect moving ball if not in ally robot
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

            return ComponentPosition(td.robot.getFrame()).positionObject(wm, td);
        });

    // Wait for the ball with the correct rotation
    DriveStep wait_for_kick;
    wait_for_kick.setRotationControl(HeadingRotationControl("ball"));
    wait_for_kick.setCancelCondition(ball_moving);
    wait_for_kick.setReachCondition(DriveStep::ReachCondition::NEVER);
    addStep(std::move(wait_for_kick));

    addStep(DribblerStep(robot_interface::DribblerMode::OFF));
    addStep(KickStep(7.0, false, robot_interface::KickExecuteTime::WHEN_BALL_IN_DRIBBLER));

    DriveStep receive_ball;
    receive_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    receive_ball.addFeature(BallTargetFeature(LineShape(past_ball_pos, "ball", -10.0, -10.0), 1.0, true, 0.24, 0.016));
    receive_ball.setRotationControl(HeadingRotationControl("ball"));

    addStep(std::move(receive_ball));

    // end of skill
}
}  // namespace luhsoccer::skills
