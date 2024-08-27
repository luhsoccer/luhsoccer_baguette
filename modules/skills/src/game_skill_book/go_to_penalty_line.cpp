#include "skill_books/game_skill_book/go_to_penalty_line.hpp"
#include "robot_control/components/component_data.hpp"
#include "robot_control/components/features/anti_target_feature.hpp"
#include "robot_control/components/features/obstacle_feature.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/circle_shape.hpp"
#include "robot_control/components/shapes/line_shape.hpp"
#include "robot_control/components/shapes/point_shape.hpp"
#include "robot_control/components/steps/dribbler_step.hpp"
#include "robot_control/components/steps/drive_step.hpp"
// include components here

namespace luhsoccer::skills {

GoToPenaltyLineBuild::GoToPenaltyLineBuild()
    : SkillBuilder("GoToPenaltyLine",  //
                   {},                 //
                   {},                 //
                   {},                 //
                   {},                 //
                   {"On ally goal"},   //
                   {}){};

void GoToPenaltyLineBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    DriveStep drive_to_line;

    // // loop through all allies and enemies for target features
    for (const auto other_robot : transform::RobotDataStorage::generateAllPossibleRobots(MAX_ROBOTS_PER_TEAM)) {
        DoubleComponentParam w(CALLBACK, [other_robot](const ComponentData& comp_data, const ComponentUid&) -> double {
            auto other_robot_data = comp_data.wm->getRobotData(other_robot);

            if (other_robot != comp_data.td.robot && other_robot_data.has_value() && other_robot_data->on_field) {
                return 1.0;
            } else {
                return 0.0;
            }
        });
        drive_to_line.addFeature(
            ObstacleFeature(CircleShape(other_robot, cs.robot_control_config.robot_radius, true), w));
    }

    drive_to_line.setAvoidOtherRobots(false);

    // avoid ball when driving to penalty line
    drive_to_line.addFeature(AntiTargetFeature(PointShape("ball"), 1.0));
    constexpr double PENALTY_DISTANCE_A = 9.5 - 6.0;
    constexpr double PENALTY_DISTANCE_B = 7.5 - 4.5;

    // Positions for the penalty lines on the ally side
    ComponentPosition penalty_line_ally_left(
        CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> transform::Position {
            double sign = comp_data.td.required_bools[0] ? 1.0 : -1.0;
            double x_coord =
                comp_data.wm->getFieldData().division == Division::A ? PENALTY_DISTANCE_A : PENALTY_DISTANCE_B;
            return {transform::field::MID_LINE_LEFT, sign * x_coord, 0.0, 0.0};
        });
    ComponentPosition penalty_line_ally_right(
        CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> transform::Position {
            double sign = comp_data.td.required_bools[0] ? 1.0 : -1.0;
            double x_coord =
                comp_data.wm->getFieldData().division == Division::A ? PENALTY_DISTANCE_A : PENALTY_DISTANCE_B;
            return {transform::field::MID_LINE_RIGHT, sign * x_coord, 0.0, 0.0};
        });
    drive_to_line.addFeature(TargetFeature(LineShape(penalty_line_ally_left, penalty_line_ally_right)));
    drive_to_line.setRotationControl(HeadingRotationControl(
        {CALLBACK, [](const ComponentData& comp_data,
                      const ComponentUid&) { return comp_data.td.required_bools[0] ? L_PI : 0.0; }},
        ""));

    addStep(DribblerStep(robot_interface::DribblerMode::OFF));
    addStep(std::move(drive_to_line));
    // end of skill
}
}  // namespace luhsoccer::skills