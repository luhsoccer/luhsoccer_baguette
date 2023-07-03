#include "skill_books/bod_skill_book/receive_ball.hpp"
#include "local_planner_components/features/ball_target_feature.hpp"
#include "local_planner_components/steps/condition_step.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/steps/wait_step.hpp"

namespace luhsoccer::skills {

ReceiveBallBuild::ReceiveBallBuild()
    : SkillBuilder("ReceiveBall",  //
                   {},             //
                   {},             //
                   {},             //
                   {},             //
                   {},             //
                   {}){};

void ReceiveBallBuild::buildImpl(const config_provider::ConfigStore& cs) {
    addStep(DribblerStep(robot_interface::DribblerMode::LOW));

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

    // Wait for the ball with the correct rotation
    DriveStep wait_for_kick;
    wait_for_kick.setRotationControl(HeadingRotationControl("ball"));
    wait_for_kick.setCancelCondition(ball_moving);
    wait_for_kick.setReachCondition(DriveStep::ReachCondition::NEVER);
    addStep(std::move(wait_for_kick));

    // DriveStep for future ball position
    DriveStep receive_ball;
    receive_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);

    // improvise a LineShape using the past ball position
    ComponentPosition past_ball_pos(
        CALLBACK,
        [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> transform::Position {
            ComponentPosition ball_pos("ball");
            auto past_ball_pos = ball_pos.positionObject(wm, td).getVelocity(wm);

            // try to stabilize the line shape by using the ball velocity and not the avg of the last positions
            if (past_ball_pos.has_value() && past_ball_pos->norm() > 0.5) {
                transform::Position ball_vel(
                    "", ball_pos.positionObject(wm, td).getCurrentPosition(wm)->translation().x() - past_ball_pos->x(),
                    ball_pos.positionObject(wm, td).getCurrentPosition(wm)->translation().y() - past_ball_pos->y());
                return ball_vel;
            }

            return ComponentPosition(td.robot.getFrame()).positionObject(wm, td);
        });
    receive_ball.addFeature(BallTargetFeature(LineShape(past_ball_pos, "ball", -10.0, -10.0), 1.0, true,
                                              cs.skills_config.receive_ball_kg, cs.skills_config.receive_ball_kv));
    receive_ball.setRotationControl(HeadingRotationControl("ball"));

    addStep(std::move(receive_ball));

    addStep(DribblerStep(robot_interface::DribblerMode::HIGH));

    // end of skill
}
}  // namespace luhsoccer::skills