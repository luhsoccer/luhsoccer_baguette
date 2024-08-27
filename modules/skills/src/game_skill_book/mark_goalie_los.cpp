#include "skill_books/game_skill_book/mark_goalie_los.hpp"
#include "config/skills_config.hpp"
#include "robot_control/components/component_data.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/circle_shape.hpp"
#include "robot_control/components/shapes/line_shape.hpp"
#include "robot_control/components/steps/dribbler_step.hpp"
#include "robot_control/components/steps/drive_step.hpp"
#include "robot_control/components/steps/kick_step.hpp"
// include components here

namespace luhsoccer::skills {

MarkGoalieLosBuild::MarkGoalieLosBuild()
    : SkillBuilder("MarkGoalieLos",   //
                   {"Enemy goalie"},  //
                   {},                //
                   {},                //
                   {},                //
                   {},                //
                   {}){};

void MarkGoalieLosBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    addStep(DribblerStep(robot_interface::DribblerMode::HIGH));
    addStep(KickStep(MAX_KICK_VELOCITY, false, robot_interface::KickExecuteTime::WHEN_BALL_IN_DRIBBLER));

    DriveStep drive_to_line_of_sight;

    drive_to_line_of_sight.setReachCondition(DriveStep::ReachCondition::NEVER);

    drive_to_line_of_sight.setRotationControl(HeadingRotationControl("ball"));

    // LineShape goalie to ball
    constexpr double MAX_INTERCEPTION_DISTANCE = 7.0;
    drive_to_line_of_sight.addFeature(
        TargetFeature(LineShape("ball", {TD_Pos::ROBOT, 0}, -MAX_INTERCEPTION_DISTANCE, 0.0)));
    constexpr double RADIUS_OFFSET = 0.2;
    DoubleComponentParam radius(CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> double {
        auto field_data = comp_data.wm->getFieldData();
        return Eigen::Vector2d(field_data.penalty_area_depth, field_data.penalty_area_width / 2).norm() + RADIUS_OFFSET;
    });

    TargetFeature circle_target(CircleShape({transform::field::GOAL_ENEMY_CENTER}, radius, false));
    circle_target.setWeight(cs.skills_config.marking_distance_weight);
    drive_to_line_of_sight.addFeature(std::move(circle_target));

    addStep(std::move(drive_to_line_of_sight));
    // end of skill
}
}  // namespace luhsoccer::skills