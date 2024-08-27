#include "skill_books/game_skill_book/mark_enemy_to_ball.hpp"
#include "config/skills_config.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/circle_shape.hpp"
#include "robot_control/components/shapes/line_shape.hpp"
#include "robot_control/components/steps/dribbler_step.hpp"
#include "robot_control/components/steps/drive_step.hpp"
// include components here

namespace luhsoccer::skills {

MarkEnemyToBallBuild::MarkEnemyToBallBuild()
    : SkillBuilder("MarkEnemyToBall",  //
                   {"Enemy"},          //
                   {},                 //
                   {},                 //
                   {},                 //
                   {},                 //
                   {}){};

void MarkEnemyToBallBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    DriveStep mark_enemy_step;

    addStep(DribblerStep(robot_interface::DribblerMode::OFF));

    mark_enemy_step.addFeature(
        TargetFeature(LineShape({TD_Pos::ROBOT, 0}, {"ball"}, cs.skills_config.marking_enemy_radius)));
    TargetFeature c_t(CircleShape({TD_Pos::ROBOT, 0}, cs.skills_config.marking_enemy_radius, false));
    c_t.setWeight(cs.skills_config.marking_distance_weight);
    mark_enemy_step.addFeature(std::move(c_t));

    mark_enemy_step.setRotationControl(HeadingRotationControl({"ball"}, false));

    // add conditions
    mark_enemy_step.setReachCondition(DriveStep::ReachCondition::NEVER);

    // add step
    addStep(std::move(mark_enemy_step));
    // end of skill
}
}  // namespace luhsoccer::skills