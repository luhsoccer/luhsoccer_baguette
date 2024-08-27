#include "skill_books/game_skill_book/place_ball.hpp"
#include "ball_steps.hpp"
#include "config/skills_config.hpp"
#include "robot_control/components/component_data.hpp"
#include "robot_control/components/features/anti_target_feature.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/features/turn_around_ball_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/circle_shape.hpp"
#include "robot_control/components/shapes/line_shape.hpp"
#include "robot_control/components/shapes/point_shape.hpp"
#include "robot_control/components/steps/condition_step.hpp"
#include "robot_control/components/steps/dribbler_step.hpp"
#include "robot_control/components/steps/drive_step.hpp"
#include "robot_control/components/steps/turn_around_ball_step.hpp"
#include "robot_control/components/steps/wait_step.hpp"
#include "ball_steps.hpp"

namespace luhsoccer::skills {

PlaceBallBuild::PlaceBallBuild()
    : SkillBuilder("PlaceBall",  //
                   {},           //
                   {"Target"},   //
                   {},           //
                   {},           //
                   {},           //
                   {}){};

void PlaceBallBuild::buildImpl(const config_provider::ConfigStore& cs) {
    ComponentPosition target_pos({TD_Pos::POINT, 0});
    constexpr double EDGE_SAFETY_MARGIN = 0.6;

    addStep(DribblerStep(robot_interface::DribblerMode::OFF));

    // only start when the ball is not moving
    DriveStep wait_for_ball_stopped;
    wait_for_ball_stopped.addFeature(TargetFeature(CircleShape("ball", 1.0, false)));
    wait_for_ball_stopped.setRotationControl(HeadingRotationControl("ball"));

    wait_for_ball_stopped.setReachCondition(DriveStep::ReachCondition::NEVER);
    wait_for_ball_stopped.setCancelCondition(
        {CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> bool {
             auto ball_vel = comp_data.wm->getVelocity(comp_data.wm->getBallFrame(), "", "", time::TimePoint(0),
                                                       time::Duration(1.0));
             if (ball_vel.has_value() && ball_vel->velocity.norm() < 0.15 && ball_vel->velocity.norm() > 0.00001) {
                 return true;
             }
             return false;
         }});
    addStep(std::move(wait_for_ball_stopped));

    // check if ball is too close to ball -> move ball away from wall
    ConditionStep ball_too_close(
        {CALLBACK, [&cs](const ComponentData& comp_data, const ComponentUid&) -> bool {
             auto ball_pos = ComponentPosition("ball").positionObject(comp_data).getCurrentPosition(comp_data.wm);

             if (ball_pos.has_value()) {
                 auto field_size = comp_data.wm->getFieldData().size;

                 if (ball_pos->translation().x() <
                         -field_size.x() / 2 + cs.skills_config.placeball_skill_ball_too_close_margin ||
                     ball_pos->translation().x() >
                         field_size.x() / 2 - cs.skills_config.placeball_skill_ball_too_close_margin ||
                     ball_pos->translation().y() >
                         field_size.y() / 2 - cs.skills_config.placeball_skill_ball_too_close_margin ||
                     ball_pos->translation().y() <
                         -field_size.y() / 2 + cs.skills_config.placeball_skill_ball_too_close_margin) {
                     return true;
                 }
             }

             return false;
         }});

    // IfStep: move ball away from wall
    ComponentPosition move_ball_away_pos(
        CALLBACK, [&cs](const ComponentData& comp_data, const ComponentUid&) -> transform::Position {
            auto ball_pos = ComponentPosition("ball").positionObject(comp_data).getCurrentPosition(comp_data.wm);

            if (ball_pos.has_value()) {
                auto field_size = comp_data.wm->getFieldData().size;

                double target_x = 0.0;
                double target_y = 0.0;

                if (ball_pos->translation().x() <
                    -field_size.x() / 2 + cs.skills_config.placeball_skill_ball_too_close_margin)  // left
                    target_x = ball_pos->translation().x() + EDGE_SAFETY_MARGIN;

                if (ball_pos->translation().x() >
                    field_size.x() / 2 - cs.skills_config.placeball_skill_ball_too_close_margin)  // right
                    target_x = ball_pos->translation().x() - EDGE_SAFETY_MARGIN;

                if (ball_pos->translation().y() >
                    field_size.y() / 2 - cs.skills_config.placeball_skill_ball_too_close_margin)  // top
                    target_y = ball_pos->translation().y() - EDGE_SAFETY_MARGIN;

                if (ball_pos->translation().y() <
                    -field_size.y() / 2 + cs.skills_config.placeball_skill_ball_too_close_margin)  // bottom
                    target_y = ball_pos->translation().y() + EDGE_SAFETY_MARGIN;

                if (target_x != 0.0 || target_y != 0.0) {
                    if (target_x == 0.0)  //
                        target_x = ball_pos->translation().x();

                    if (target_y == 0.0)  //
                        target_y = ball_pos->translation().y();
                }

                return {"", target_x, target_y};
            }

            return {"", 0.0, 0.0};
        });

    DriveStep move_to_ball;
    move_to_ball.addFeature(TargetFeature(CircleShape("ball", 0.3, false)));
    move_to_ball.addFeature(TargetFeature(LineShape("ball", move_ball_away_pos)));
    move_to_ball.setRotationControl(HeadingRotationControl("ball"));

    move_to_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    move_to_ball.setAvoidDefenseArea(false);
    move_to_ball.setAvoidOtherRobots(false);
    move_to_ball.activateConstraint(DriveStepConstraintNames::BALL_PLACEMENT, false);
    move_to_ball.activateConstraint(DriveStepConstraintNames::BOUNDARIES, false);
    ball_too_close.addIfStep(std::move(move_to_ball));

    DriveStep drive_into_ball;
    drive_into_ball.addFeature(
        BallTargetFeature(CircleShape("ball", 0.07, false), 1.0, false, cs.skills_config.ball_get_ball_k_p_scaling));
    drive_into_ball.setRotationControl(HeadingRotationControl("ball"));
    drive_into_ball.setCancelCondition({CALLBACK, [](const CallbackData& data) -> bool {
                                            auto ball_info = data.wm->getBallInfo();

                                            if (ball_info.has_value()) {
                                                return ball_info->isInRobot(data.td.robot);
                                            }

                                            return false;
                                        }});

    drive_into_ball.setMaxVelX(0.25);
    drive_into_ball.setMaxVelY(0.25);
    drive_into_ball.setReachCondition(DriveStep::ReachCondition::NEVER);
    drive_into_ball.setAvoidDefenseArea(false);
    drive_into_ball.setAvoidOtherRobots(false);
    drive_into_ball.activateConstraint(DriveStepConstraintNames::BOUNDARIES, false);
    drive_into_ball.activateConstraint(DriveStepConstraintNames::BALL_PLACEMENT, false);
    ball_too_close.addIfStep(DribblerStep(robot_interface::DribblerMode::LOW));
    ball_too_close.addIfStep(std::move(drive_into_ball));
    ball_too_close.addIfStep(WaitStep(WAIT_DURATION, 0.5));

    DriveStep drive_backwards;
    drive_backwards.addFeature(TargetFeature(PointShape(move_ball_away_pos)));
    drive_backwards.setRotationControl(HeadingRotationControl("ball"));

    drive_backwards.setMaxVelX(0.5);
    drive_backwards.setMaxVelY(0.5);

    drive_backwards.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    drive_backwards.setAvoidDefenseArea(false);
    drive_backwards.setAvoidOtherRobots(false);
    drive_backwards.activateConstraint(DriveStepConstraintNames::BALL_PLACEMENT, false);
    drive_backwards.activateConstraint(DriveStepConstraintNames::BOUNDARIES, false);
    drive_backwards.setCancelCondition(
        {CALLBACK, [&cs](const ComponentData& comp_data, const ComponentUid&) -> bool {
             auto ball_pos = ComponentPosition("ball").positionObject(comp_data).getCurrentPosition(comp_data.wm);

             if (ball_pos.has_value()) {
                 auto field_size = comp_data.wm->getFieldData().size;

                 double dist_left = ball_pos->translation().x() + field_size.x() / 2;
                 double dist_right = field_size.x() / 2 - ball_pos->translation().x();
                 double dist_top = field_size.y() / 2 - ball_pos->translation().y();
                 double dist_bottom = ball_pos->translation().y() + field_size.y() / 2;

                 if (dist_left > cs.skills_config.placeball_skill_ball_too_close_margin &&
                     dist_right > cs.skills_config.placeball_skill_ball_too_close_margin &&
                     dist_top > cs.skills_config.placeball_skill_ball_too_close_margin &&
                     dist_bottom > cs.skills_config.placeball_skill_ball_too_close_margin) {
                     return true;
                 }
             }

             return false;
         }});
    ball_too_close.addIfStep(std::move(drive_backwards));
    ball_too_close.addIfStep(DribblerStep(robot_interface::DribblerMode::OFF));
    ball_too_close.addIfStep(WaitStep(WAIT_DURATION, 1.25));

    addStep(std::move(ball_too_close));

    // preposition for turn around ball
    DriveStep prepare_for_turn_around_ball;
    prepare_for_turn_around_ball.addFeature(TargetFeature(CircleShape("ball", 0.3, false)));
    prepare_for_turn_around_ball.addFeature(
        AntiTargetFeature(PointShape("ball"), 0.3));  // remove if new ball filter works?
    prepare_for_turn_around_ball.setRotationControl(HeadingRotationControl("ball"));

    prepare_for_turn_around_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    prepare_for_turn_around_ball.setAvoidDefenseArea(false);
    prepare_for_turn_around_ball.setAvoidOtherRobots(false);
    prepare_for_turn_around_ball.activateConstraint(DriveStepConstraintNames::BALL_PLACEMENT, false);
    addStep(std::move(prepare_for_turn_around_ball));

    // turn around ball
    TurnAroundBallStep turn_around_ball(target_pos);
    turn_around_ball.setAvoidDefenseArea(false);
    turn_around_ball.setAvoidOtherRobots(false);
    addStep(std::move(turn_around_ball));

    ConditionStep ball_in_dribbler({CALLBACK, [](const CallbackData& data) -> bool {
                                        auto ball_data = data.wm->getBallInfo();
                                        return ball_data.has_value() && ball_data->isInRobot(data.td.robot);
                                    }});

    auto get_ball = getGetBallStep(cs);
    get_ball->activateConstraint(DriveStepConstraintNames::BALL_PLACEMENT, false);
    get_ball->setAvoidDefenseArea(false);
    get_ball->setAvoidOtherRobots(false);
    ball_in_dribbler.addElseStepPtr(get_ball);

    addStep(DribblerStep(robot_interface::DribblerMode::LOW));
    addStep(std::move(ball_in_dribbler));

    // move to target_pos with ball
    DriveStep move_to_target;
    move_to_target.addFeature(TargetFeature(CircleShape(target_pos, 0.09, false)));
    move_to_target.setRotationControl(HeadingRotationControl(target_pos));

    move_to_target.setMaxVelX(1.0);
    move_to_target.setMaxVelY(1.0);
    move_to_target.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    move_to_target.setAvoidDefenseArea(false);
    move_to_target.setAvoidOtherRobots(false);
    move_to_target.activateConstraint(DriveStepConstraintNames::BALL_PLACEMENT, false);
    addStep(std::move(move_to_target));

    // wait at target_pos
    addStep(DribblerStep(robot_interface::DribblerMode::OFF));
    addStep(WaitStep(WAIT_DURATION, 2.0));

    // finish place ball by driving away
    DriveStep drive_away;
    drive_away.addFeature(TargetFeature(CircleShape("ball", 0.8, false)));
    drive_away.setRotationControl(HeadingRotationControl("ball"));

    drive_away.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    drive_away.setAvoidDefenseArea(false);
    drive_away.setAvoidOtherRobots(false);
    drive_away.activateConstraint(DriveStepConstraintNames::BALL_PLACEMENT, false);
    addStep(std::move(drive_away));

    addStep(DribblerStep(robot_interface::DribblerMode::OFF));

    // [TODO]: replace constants
}
}  // namespace luhsoccer::skills