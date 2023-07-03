#include "skill_books/bod_skill_book/go_to_point_intercept.hpp"
#include "local_planner_components/features/ball_target_feature.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/rotation_controls/rotate_to_move_direction_control.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/steps/condition_step.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/steps/drive_step.hpp"
// include components here

namespace luhsoccer::skills {

GoToPointInterceptBuild::GoToPointInterceptBuild()
    : SkillBuilder("GoToPointIntercept",  //
                   {},                    //
                   {"TargetPose"},        //
                   {},                    //
                   {},                    //
                   {},                    //
                   {}){};

void GoToPointInterceptBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    BoolComponentParam face_away(CALLBACK, [](const CallbackData& data) -> bool {
        auto target_in_robot_frame =
            data.td.required_positions[0].getCurrentPosition(data.wm, data.td.robot.getFrame());
        return target_in_robot_frame.has_value() && target_in_robot_frame->translation().x() < 0.0;
    });
    DriveStep turn_to_target;
    turn_to_target.addFeature(TargetFeature(PointShape({TD_Pos::EXECUTING_ROBOT, 0}), 1000.0));
    turn_to_target.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    turn_to_target.setRotationControl(HeadingRotationControl({TD_Pos::POINT, 0}, face_away, 2.0));

    DriveStep drive_to_target;
    drive_to_target.addFeature(TargetFeature(PointShape({TD_Pos::POINT, 0}), 1.0));
    drive_to_target.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    drive_to_target.setRotationControl(RotateToMoveDirectionControl(face_away));

    // turn to final heading
    DriveStep turn;
    turn.addFeature(TargetFeature(PointShape({TD_Pos::POINT, 0})));
    turn.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    turn.setRotationControl(HeadingRotationControl("ball"));

    constexpr double DISTANCE_FOR_ALIGNED_MOVEMENT = 2.0;
    ConditionStep cond(
        {CALLBACK, [](const CallbackData& data) -> bool {
             auto goal_transform = data.td.required_positions[0].getCurrentPosition(data.wm, data.td.robot.getFrame());
             return goal_transform.has_value() && goal_transform->translation().norm() > DISTANCE_FOR_ALIGNED_MOVEMENT;
         }});
    cond.addIfStep(std::move(turn_to_target));
    cond.addIfStep(std::move(drive_to_target));
    cond.addIfStep(std::move(turn));

    DriveStep drive_direct;
    drive_direct.addFeature(TargetFeature(PointShape({TD_Pos::POINT, 0})));
    drive_direct.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    drive_direct.setRotationControl(HeadingRotationControl("ball"));

    cond.addElseStep(std::move(drive_direct));
    addStep(std::move(cond));

    // Intercept
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
    receive_ball.setRotationControl(HeadingRotationControl({TD_Pos::EXECUTING_ROBOT, 0}));

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