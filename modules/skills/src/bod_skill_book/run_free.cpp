#include "skill_books/bod_skill_book/run_free.hpp"
#include "local_planner_components/features/anti_target_feature.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/shapes/rectangle_shape.hpp"
#include "local_planner_components/steps/drive_step.hpp"
// include components here

namespace luhsoccer::skills {

RunFreeBuild::RunFreeBuild()
    : SkillBuilder("RunFree",        //
                   {"enemy_robot"},  //
                   {"area_center"},  //
                   {"radius"},       //
                   {},               //
                   {},               //
                   {}){};

void RunFreeBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step

    DriveStep run_free;
    DoubleComponentParam circle_area_weight(
        CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            DoubleComponentParam radius_comp = {TD, 0};
            ComponentPosition robot(td.robot.getFrame());
            ComponentPosition circle_center = {TD_Pos::POINT, 0};

            auto robot_pos = robot.positionObject(wm, td).getCurrentPosition(wm);
            auto center_pos = circle_center.positionObject(wm, td).getCurrentPosition(wm);

            if (!robot_pos.has_value() || !center_pos.has_value()) return 1;

            if ((robot_pos->translation() - center_pos->translation()).norm() > radius_comp.val(wm, td)) {
                return 0;
            }

            return 1.0;
        });
    run_free.addFeature(AntiTargetFeature(CircleShape({TD_Pos::POINT, 0}, {TD, 0}, {false}),
                                          {cs.skills_config.rf_influence_distance_circle}, circle_area_weight));
    run_free.addFeature(AntiTargetFeature(CircleShape({TD_Pos::ROBOT, 0}, {cs.skills_config.rf_enemy_radius}, {true}),
                                          {cs.skills_config.rf_influence_distance_enemy_robot}));
    run_free.addFeature(TargetFeature(PointShape({TD_Pos::POINT, 0}), {DEFAULT_TRANSLATIONAL_TOLERANCE},
                                      {cs.skills_config.rf_weight_center}));
    run_free.setReachCondition(DriveStep::ReachCondition::NEVER);
    run_free.setRotationControl(HeadingRotationControl("ball"));

    addStep(std::move(run_free));
    // end of skill
}
}  // namespace luhsoccer::skills