#include "skill_books/bod_skill_book/mark_enemy_to_goal.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "config/skills_config.hpp"

namespace luhsoccer::skills {

MarkEnemyToGoalBuild::MarkEnemyToGoalBuild()
    : SkillBuilder("MarkEnemyToGoal",  //
                   {"EnemyRobot"},     //
                   {},                 //
                   {},                 //
                   {},                 //
                   {},                 //
                   {}){};

void MarkEnemyToGoalBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // DriveStep mark_enemy_step;
    auto mark_enemy_step = DriveStep();

    // DribblerStep
    addStep(DribblerStep(robot_interface::DribblerMode::OFF));

    // define parameters
    ComponentPosition goal_centre(transform::field::GOAL_ALLY_CENTER);
    ComponentPosition enemy(TD_Pos::ROBOT, 0);
    DoubleComponentParam defend_radius(cs.skills_config.mark_radius_enemy);

    // add features
    mark_enemy_step.addFeature(TargetFeature(LineShape(enemy, goal_centre, {defend_radius})));
    mark_enemy_step.addFeature(TargetFeature(CircleShape(enemy, defend_radius, false)));

    mark_enemy_step.setRotationControl(HeadingRotationControl(enemy));

    // add conditions
    mark_enemy_step.setReachCondition(DriveStep::ReachCondition::NEVER);

    // add step
    addStep(std::move(mark_enemy_step));
    // end of skill
}
}  // namespace luhsoccer::skills
