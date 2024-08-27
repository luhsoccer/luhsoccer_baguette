#include "skill_books/game_skill_book/get_ball.hpp"
#include "config/skills_config.hpp"
#include "go_to_ball_steps.hpp"
#include "robot_control/components/component_data.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/circle_shape.hpp"
#include "robot_control/components/steps/condition_step.hpp"
#include "robot_control/components/steps/dribbler_step.hpp"
#include "robot_control/components/steps/drive_step.hpp"
#include "ball_steps.hpp"

// include components here

namespace luhsoccer::skills {

GetBallBuild::GetBallBuild()
    : SkillBuilder("GetBall",  //
                   {},         //
                   {},         //
                   {},         //
                   {},         //
                   {},         //
                   {}){};

void GetBallBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    addStep(DribblerStep(robot_interface::DribblerMode::LOW));

    ConditionStep c({CALLBACK, [&cs](const ComponentData& comp_data, const ComponentUid&) {
                         auto ball_in_robot_pos = transform::Position("ball").getCurrentPosition(
                             comp_data.wm, {comp_data.td.robot.getFrame()}, comp_data.time);

                         if (!ball_in_robot_pos.has_value()) return false;
                         return ball_in_robot_pos->translation().norm() > cs.skills_config.ball_distance_for_pre_drive;
                     }});

    DriveStep go_to_get_ball;
    go_to_get_ball.setRotationControl(HeadingRotationControl("ball", false));
    TargetFeature t(CircleShape("ball", cs.skills_config.ball_go_to_ball_radius, true));
    t.setKp(cs.skills_config.ball_get_ball_k_p_scaling);
    t.setVelocityZeroForReach(false);
    go_to_get_ball.addFeature(std::move(t));
    c.addIfStep(std::move(go_to_get_ball));
    addStep(std::move(c));

    // drive even closer to ball
    addStepPtr(getGetBallStep(cs));
    // end of skill
}
}  // namespace luhsoccer::skills