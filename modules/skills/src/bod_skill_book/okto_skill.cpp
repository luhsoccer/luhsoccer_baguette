#include "skill_books/bod_skill_book/okto_skill.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/shapes/rectangle_shape.hpp"
// include components here

namespace luhsoccer::skills {

OktoSkillBuild::OktoSkillBuild()
    : SkillBuilder("OktoSkill",     //
                   {},              //
                   {},              //
                   {"v_x", "v_y"},  //
                   {},              //
                   {},              //
                   {}){};

void OktoSkillBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    ComponentPosition target_pos(
        CALLBACK,
        [&cs](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> transform::Position {
            ComponentPosition robot_comp(td.robot.getFrame());
            auto robot_pos = robot_comp.positionObject(wm, td).getCurrentPosition(wm);
            if (!robot_pos.has_value()) return {""};

            auto corner_enemy_pos =
                ComponentPosition("field_corner_enemy_left").positionObject(wm, td).getCurrentPosition(wm);
            auto corner_ally_pos =
                ComponentPosition("field_corner_ally_right").positionObject(wm, td).getCurrentPosition(wm);

            if (!corner_enemy_pos.has_value() || !corner_ally_pos.has_value()) return {""};

            double add_x = DoubleComponentParam(TD, 0).val(wm, td);
            double add_y = DoubleComponentParam(TD, 1).val(wm, td);

            if (std::abs(corner_ally_pos->translation().x() - robot_pos->translation().x()) <
                    cs.skills_config.okto_skill_dist &&
                add_x < 0) {
                add_x = 0;
            }
            if (std::abs(corner_enemy_pos->translation().x() - robot_pos->translation().x()) <
                    cs.skills_config.okto_skill_dist &&
                add_x > 0) {
                add_x = 0;
            }
            if (std::abs(corner_ally_pos->translation().y() - robot_pos->translation().y()) <
                    cs.skills_config.okto_skill_dist &&
                add_y < 0) {
                add_y = 0;
            }
            if (std::abs(corner_enemy_pos->translation().y() - robot_pos->translation().y()) <
                    cs.skills_config.okto_skill_dist &&
                add_y > 0) {
                add_y = 0;
            }

            return {"", robot_pos->translation().x() + add_x, robot_pos->translation().y() + add_y};
        });
    ComponentPosition robot(TD_Pos::EXECUTING_ROBOT, 0);

    DriveStep okto_drive;
    okto_drive.setReachCondition(DriveStep::ReachCondition::NEVER);
    okto_drive.setRotationControl(HeadingRotationControl("ball"));
    okto_drive.addFeature(TargetFeature(PointShape(target_pos)));

    addStep(std::move(okto_drive));
    // end of skill
}
}  // namespace luhsoccer::skills