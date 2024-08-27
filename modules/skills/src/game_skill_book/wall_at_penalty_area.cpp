#include "skill_books/game_skill_book/wall_at_penalty_area.hpp"
#include "config/robot_control_config.hpp"
#include "config/skills_config.hpp"
#include "robot_control/components/component_data.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/compose_shape.hpp"
#include "robot_control/components/shapes/line_shape.hpp"
#include "robot_control/components/steps/dribbler_step.hpp"
#include "robot_control/components/steps/drive_step.hpp"
// include components here

namespace luhsoccer::skills {

WallAtPenaltyAreaBuild::WallAtPenaltyAreaBuild()
    : SkillBuilder("WallAtPenaltyArea",                              //
                   {"VARIADIC"},                                     //
                   {},                                               //
                   {},                                               //
                   {"DefendingRobotIndex", "TotalDefendingRobots"},  //
                   {},                                               //
                   {}){};

void WallAtPenaltyAreaBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // DribblerStep
    addStep(DribblerStep(robot_interface::DribblerMode::OFF));

    // defense area shape
    ComposeShape defense_area;
    ComponentPosition defense_area_corner_left(CALLBACK, [&cs]() -> transform::Position {
        return {transform::field::DEFENSE_AREA_CORNER_ALLY_LEFT, cs.skills_config.marking_wall_defense_area_margin,
                cs.skills_config.marking_wall_defense_area_margin};
    });
    ComponentPosition defense_area_intersection_left(CALLBACK, [&cs]() -> transform::Position {
        return {transform::field::DEFENSE_AREA_INTERSECTION_ALLY_LEFT, 0.0,
                cs.skills_config.marking_wall_defense_area_margin};
    });
    ComponentPosition defense_area_corner_right(CALLBACK, [&cs]() -> transform::Position {
        return {transform::field::DEFENSE_AREA_CORNER_ALLY_RIGHT, cs.skills_config.marking_wall_defense_area_margin,
                -cs.skills_config.marking_wall_defense_area_margin};
    });
    ComponentPosition defense_area_intersection_right(CALLBACK, [&cs]() -> transform::Position {
        return {transform::field::DEFENSE_AREA_INTERSECTION_ALLY_RIGHT, 0.0,
                -cs.skills_config.marking_wall_defense_area_margin};
    });

    defense_area.addShape(LineShape({defense_area_intersection_left}, {defense_area_corner_left}));
    defense_area.addShape(LineShape({defense_area_corner_left}, {defense_area_corner_right}));
    defense_area.addShape(LineShape({defense_area_intersection_right}, {defense_area_corner_right}));

    // weighted enemy position
    ComponentPosition enemy_center_pose(
        CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> transform::Position {
            Eigen::Vector2d v_enemy_center = {0.0, 0.0};
            double weight_sum = 0;
            for (const auto& robot : comp_data.td.related_robots) {
                if (robot != EMPTY_IDENTIFIER) {
                    std::optional<Eigen::Affine2d> pos_enemy =
                        transform::Position(robot.getFrame()).getCurrentPosition(comp_data.wm, "", comp_data.time);
                    if (pos_enemy.has_value()) {
                        weight_sum++;
                        v_enemy_center += pos_enemy->translation();
                    }
                }
            }
            if (weight_sum != 0) {
                Eigen::Translation2d t_enemy_center(v_enemy_center / weight_sum);
                return {"", t_enemy_center * Eigen::Rotation2Dd(0)};
            }
            return {""};
        });

    auto calc_offset = [&cs, enemy_center_pose](const ComponentData& comp_data) -> std::optional<Eigen::Vector2d> {
        // generate v_off
        std::optional<Eigen::Affine2d> x_goal_enemy = enemy_center_pose.positionObject(comp_data).getCurrentPosition(
            comp_data.wm, transform::field::GOAL_ALLY_CENTER, comp_data.time);
        if (x_goal_enemy.has_value()) {
            // switched intentionally
            double enemy_angle = std::atan2(x_goal_enemy->translation().y(), x_goal_enemy->translation().x());
            double offset = -(comp_data.td.required_ints[1] - 1.0) * cs.robot_control_config.robot_radius;
            offset += comp_data.td.required_ints[0] * cs.robot_control_config.robot_radius * 2;
            Eigen::Vector2d v_off = {0.0, 0.0};
            v_off.y() += offset;
            Eigen::Vector2d v_off_rot = Eigen::Rotation2Dd(enemy_angle) * v_off;
            return v_off_rot;
        } else {
            return std::nullopt;
        }
    };

    ComponentPosition enemy_center_pose_offset(
        CALLBACK,
        [enemy_center_pose, calc_offset](const ComponentData& comp_data, const ComponentUid&) -> transform::Position {
            // add v_off to enemy_center_pose
            std::optional<Eigen::Affine2d> x_enemy_center =
                enemy_center_pose.positionObject(comp_data).getCurrentPosition(comp_data.wm, "", comp_data.time);
            auto v_off = calc_offset(comp_data);
            if (x_enemy_center.has_value() && v_off.has_value()) {
                Eigen::Translation2d t_enemy_center(x_enemy_center->translation() + v_off.value());
                return {"", t_enemy_center * Eigen::Rotation2Dd(0)};
            }

            return {""};
        });

    // ComponentPosition enemy_pose_offset;
    ComponentPosition goal_center_offset(
        CALLBACK, [calc_offset](const ComponentData& comp_data, const ComponentUid&) -> transform::Position {
            auto v_off = calc_offset(comp_data);
            if (v_off.has_value()) {
                return {transform::field::GOAL_ALLY_CENTER, v_off->x(), v_off->y()};
            } else {
                return {transform::field::GOAL_ALLY_CENTER};
            }
        });

    // DriveStep penalty_wall step
    DriveStep penalty_wall;

    // add features
    penalty_wall.addFeature(TargetFeature(LineShape(enemy_center_pose_offset, goal_center_offset, 0.0, 1.0)));
    penalty_wall.addFeature(TargetFeature(defense_area));
    penalty_wall.setRotationControl(HeadingRotationControl(enemy_center_pose_offset));

    // add conditions
    penalty_wall.setReachCondition(DriveStep::ReachCondition::NEVER);

    // add step
    addStep(std::move(penalty_wall));
    // end of skill
}
}  // namespace luhsoccer::skills