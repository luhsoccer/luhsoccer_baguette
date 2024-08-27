#include "skill_books/game_skill_book/steal_ball.hpp"
#include "config/skills_config.hpp"
#include "robot_control/components/component_data.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/arc_shape.hpp"
#include "robot_control/components/shapes/circle_shape.hpp"
#include "robot_control/components/shapes/point_shape.hpp"
#include "robot_control/components/steps/condition_step.hpp"
#include "robot_control/components/steps/dribbler_step.hpp"
#include "robot_control/components/steps/drive_step.hpp"
// include components here

namespace luhsoccer::skills {

StealBallBuild::StealBallBuild()
    : SkillBuilder("StealBall",       //
                   {"Ball carrier"},  //
                   {},                //
                   {},                //
                   {},                //
                   {},                //
                   {}){};

void StealBallBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    ConditionStep c({CALLBACK, [](const ComponentData& comp_data, const ComponentUid& /*td*/) -> bool {
                         auto ball_info = comp_data.wm->getBallInfo();
                         if (ball_info.has_value()) {
                             if (ball_info->state == transform::BallState::IN_ROBOT && ball_info->robot.has_value() &&
                                 ball_info->robot->isAlly()) {
                                 return false;
                             }
                         }
                         return true;
                     }});

    DriveStep go_to_ball;
    ComponentPosition ball_rotated_to_target(
        CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> transform::Position {
            std::optional<Eigen::Affine2d> target_pos = transform::Position(comp_data.td.related_robots[0].getFrame())
                                                            .getCurrentPosition(comp_data.wm, "ball", comp_data.time);
            if (target_pos.has_value()) {
                double target_rot = atan2(target_pos->translation().y(), target_pos->translation().x());
                return {"ball", 0.0, 0.0, target_rot + L_PI};
            }
            return {"ball"};
        });

    go_to_ball.setRotationControl(HeadingRotationControl("ball", false, L_PI / 2));
    go_to_ball.addFeature(
        TargetFeature(ArcShape(ball_rotated_to_target, cs.skills_config.ball_go_to_ball_radius, 0.05), 0.05));

    c.addIfStep(std::move(go_to_ball));

    c.addIfStep(DribblerStep(robot_interface::DribblerMode::HIGH));

    DriveStep steal_ball;
    steal_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    steal_ball.setAvoidOtherRobots(false);
    steal_ball.setRotationControl(HeadingRotationControl("ball"));
    steal_ball.addFeature(BallTargetFeature(PointShape("ball")));
    c.addIfStep(std::move(steal_ball));

    DriveStep move_back;
    move_back.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    move_back.setAvoidOtherRobots(true);
    move_back.setRotationControl(HeadingRotationControl({TD_Pos::ROBOT, 0}));
    move_back.addFeature(TargetFeature(CircleShape({TD_Pos::ROBOT, 0}, 0.5, false)));
    move_back.setMaxVelX(1.0);
    move_back.setMaxVelY(1.0);
    c.addIfStep(std::move(move_back));

    addStep(std::move(c));
    // end of skill
}
}  // namespace luhsoccer::skills