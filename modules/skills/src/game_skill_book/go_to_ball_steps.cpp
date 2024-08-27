#include "go_to_ball_steps.hpp"
#include "config/config_store.hpp"
#include "config/skills_config.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/circle_shape.hpp"
#include "robot_control/components/steps/drive_step.hpp"

namespace luhsoccer::robot_control {

std::vector<std::shared_ptr<const AbstractStep>> getGoToBallSteps(const config_provider::ConfigStore& cs) {
    std::vector<std::shared_ptr<const AbstractStep>> steps;
    // Add steps here
    // steps.emplace_back(std::make_shared<DribblerStep>(robot_interface::DribblerMode::OFF));
    DriveStep go_to_get_ball;
    go_to_get_ball.setRotationControl(HeadingRotationControl("ball", false));
    TargetFeature t(CircleShape("ball", cs.skills_config.ball_go_to_ball_radius, true));
    t.setKp(cs.skills_config.ball_get_ball_k_p_scaling);
    t.setVelocityZeroForReach(true);
    go_to_get_ball.addFeature(std::move(t));
    steps.emplace_back(std::make_shared<DriveStep>(std::move(go_to_get_ball)));

    return steps;
}
}  // namespace luhsoccer::robot_control