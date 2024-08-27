#include "skill_books/game_skill_book/octo_skill.hpp"
#include "config/skills_config.hpp"
#include "robot_control/components/component_data.hpp"
#include "robot_control/components/features/obstacle_feature.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/inverse_rectangle_shape.hpp"
#include "robot_control/components/shapes/point_shape.hpp"
#include "robot_control/components/steps/drive_step.hpp"
// include components here

namespace luhsoccer::skills {

OctoSkillBuild::OctoSkillBuild()
    : SkillBuilder("OctoSkill",     //
                   {},              //
                   {},              //
                   {"v_x", "v_y"},  //
                   {},              //
                   {},              //
                   {}){};

void OctoSkillBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step

    // Add skill definition here. Use addStep to add a step
    ComponentPosition target_pos(
        CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> transform::Position {
            auto robot_pos =
                transform::Position(comp_data.td.robot.getFrame()).getCurrentPosition(comp_data.wm, "", comp_data.time);
            if (!robot_pos.has_value()) return {""};

            double add_x = comp_data.td.required_doubles[0];
            double add_y = comp_data.td.required_doubles[1];

            return {"", robot_pos->translation().x() + add_x, robot_pos->translation().y() + add_y};
        });

    DriveStep octo_drive;
    octo_drive.setReachCondition(DriveStep::ReachCondition::NEVER);
    octo_drive.setRotationControl(HeadingRotationControl("ball"));
    octo_drive.addFeature(TargetFeature(PointShape(target_pos)));

    auto width = [&cs](const ComponentData& comp_data, const ComponentUid&) {
        auto field_data = comp_data.wm->getFieldData();
        return field_data.size.y() - 2 * cs.skills_config.octo_skill_border_margin;
    };

    auto height = [&cs](const ComponentData& comp_data, const ComponentUid&) {
        auto field_data = comp_data.wm->getFieldData();
        return field_data.size.x() - 2 * cs.skills_config.octo_skill_border_margin;
    };

    auto obstacle = ObstacleFeature(InverseRectangleShape("", {CALLBACK, width}, {CALLBACK, height}));
    obstacle.setAvoidCollision(false);
    obstacle.setKObstacle(0.0);
    octo_drive.addFeature(std::move(obstacle));

    octo_drive.setMaxVelX(cs.skills_config.octo_skill_max_vel);
    octo_drive.setMaxVelY(cs.skills_config.octo_skill_max_vel);

    addStep(std::move(octo_drive));
    // end of skill
}
}  // namespace luhsoccer::skills