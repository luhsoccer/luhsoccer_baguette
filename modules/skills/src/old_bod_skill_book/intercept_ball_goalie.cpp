#include "skill_books/bod_skill_book/intercept_ball_goalie.hpp"
#include "local_planner_components/features/ball_target_feature.hpp"
// #include "local_planner_components/features/robot_cf_obstacle.hpp"
#include "local_planner_components/steps/condition_step.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/steps/wait_step.hpp"
#include "config/skills_config.hpp"
// #include "local_planner_components/steps/condition_step.hpp"

namespace luhsoccer::skills {

InterceptBallGoalieBuild::InterceptBallGoalieBuild()
    : SkillBuilder("InterceptBallGoalie",  //
                   {},                     //
                   {},                     //
                   {},                     //
                   {},                     //
                   {},                     //
                   {}){};

void InterceptBallGoalieBuild::buildImpl(const config_provider::ConfigStore& cs) {
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

    DriveStep receive_ball;
    receive_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);

    receive_ball.addFeature(BallTargetFeature(LineShape(past_ball_pos, "ball", -10.0, -10.0), 1.0, true, 0.24, 0.016));
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

             //  auto ball_vel = wm->getVelocity("ball");
             //  if (ball_vel.has_value() && ball_vel->velocity.norm() < 0.5) return true;

             return false;
         }});

    addStep(std::move(receive_ball));

    addStep(DribblerStep(robot_interface::DribblerMode::LOW));

    // end of skill
}
}  // namespace luhsoccer::skills
