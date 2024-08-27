#include "skill_books/bod_skill_book/move_to_penalty_line.hpp"
#include "local_planner_components/features/anti_target_feature.hpp"
#include "local_planner_components/features/robot_cf_obstacle.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/steps/condition_step.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "config/skills_config.hpp"

namespace luhsoccer::skills {

MoveToPenaltyLineBuild::MoveToPenaltyLineBuild()
    : SkillBuilder("MoveToPenaltyLine",         //
                   {},                          //
                   {},                          //
                   {},                          //
                   {},                          //
                   {"PenaltyShootOnAllyGoal"},  //
                   {}){};

void MoveToPenaltyLineBuild::buildImpl(const config_provider::ConfigStore& cs) {
    DriveStep line_shoot_on_ally_goal;
    DriveStep line_shoot_on_enemy_goal;

    // loop through all allies and enemies for target features
    for (const auto other_robot : transform::RobotDataStorage::generateAllPossibleRobots(MAX_ROBOTS_PER_TEAM)) {
        DoubleComponentParam w(
            CALLBACK,
            [other_robot](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
                auto other_robot_data = wm->getRobotData(other_robot);

                if (other_robot != td.robot && other_robot_data.has_value() && other_robot_data->on_field) {
                    return 1.0;
                } else {
                    return 0.0;
                }
            });
        line_shoot_on_ally_goal.addFeature(
            RobotCFObstacle(CircleShape(other_robot, cs.skills_config.antigial_rad, true), w));
        line_shoot_on_enemy_goal.addFeature(
            RobotCFObstacle(CircleShape(other_robot, cs.skills_config.antigial_rad, true), w));
    }

    // avoid ball when driving to penalty line
    line_shoot_on_ally_goal.addFeature(AntiTargetFeature(PointShape("ball"), 1.0));
    line_shoot_on_enemy_goal.addFeature(AntiTargetFeature(PointShape("ball"), 1.0));

    // Positions for the penalty lines on the ally side
    ComponentPosition penalty_line_ally_left(
        CALLBACK,
        [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> transform::Position {
            if (wm->getFieldData().division == Division::A) {
                return {transform::field::CORNER_ALLY_LEFT, 9.5, 0.0, 0.0};
            }

            return {transform::field::CORNER_ALLY_LEFT, 7.5, 0.0, 0.0};
        });
    ComponentPosition penalty_line_ally_right(
        CALLBACK,
        [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> transform::Position {
            if (wm->getFieldData().division == Division::A) {
                return {transform::field::CORNER_ALLY_RIGHT, 9.5, 0.0, 0.0};
            }

            return {transform::field::CORNER_ALLY_RIGHT, 7.5, 0.0, 0.0};
        });
    line_shoot_on_ally_goal.addFeature(TargetFeature(LineShape(penalty_line_ally_left, penalty_line_ally_right)));
    line_shoot_on_ally_goal.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    line_shoot_on_ally_goal.setRotationControl(HeadingRotationControl(L_PI, ""));

    // Positions for the penalty lines on the enemy side
    ComponentPosition penalty_line_enemy_left(
        CALLBACK,
        [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> transform::Position {
            if (wm->getFieldData().division == Division::A) {
                return {transform::field::CORNER_ENEMY_LEFT, -9.5, 0.0, 0.0};
            }

            return {transform::field::CORNER_ENEMY_LEFT, -7.5, 0.0, 0.0};
        });
    ComponentPosition penalty_line_enemy_right(
        CALLBACK,
        [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> transform::Position {
            if (wm->getFieldData().division == Division::A) {
                return {transform::field::CORNER_ENEMY_RIGHT, -9.5, 0.0, 0.0};
            }

            return {transform::field::CORNER_ENEMY_RIGHT, -7.5, 0.0, 0.0};
        });
    line_shoot_on_enemy_goal.addFeature(TargetFeature(LineShape(penalty_line_enemy_left, penalty_line_enemy_right)));
    line_shoot_on_enemy_goal.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    line_shoot_on_enemy_goal.setRotationControl(HeadingRotationControl(0.0, ""));

    // add condition step
    ConditionStep cond({TD, 0});
    cond.addIfStep(std::move(line_shoot_on_ally_goal));
    cond.addElseStep(std::move(line_shoot_on_enemy_goal));

    addStep(DribblerStep(robot_interface::DribblerMode::OFF));
    addStep(std::move(cond));
}
}  // namespace luhsoccer::skills