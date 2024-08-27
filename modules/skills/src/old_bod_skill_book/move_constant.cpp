#include "skill_books/bod_skill_book/move_constant.hpp"
// include components here
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/shapes/point_shape.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"

namespace luhsoccer::skills {

MoveConstantBuild::MoveConstantBuild()
    : SkillBuilder("MoveConstant",      //
                   {},                  //
                   {"TargetPosition"},  //
                   {"TargetTime"},      //
                   {},                  //
                   {},                  //
                   {}){};

void MoveConstantBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    DriveStep d;
    d.addFeature(TargetFeature(PointShape({TD_Pos::POINT, 0})));
    d.setRotationControl(HeadingRotationControl(0.0, {TD_Pos::POINT, 0}));
    d.setAvoidOtherRobots(false);
    d.setAvoidDefenseArea(false);
    DoubleComponentParam max_speed(CALLBACK, [](const CallbackData& data) -> double {
        auto robot_to_goal = data.td.required_positions[0].getCurrentPosition(data.wm, data.td.robot.getFrame());
        if (!robot_to_goal.has_value()) return 0.0;
        double distance_to_target = robot_to_goal->translation().norm();
        double time_to_target = data.td.required_doubles[0] - time::now().asSec();
        return std::max(distance_to_target / time_to_target, 0.0);
    });
    d.setMaxVelX(max_speed);
    d.setMaxVelY(max_speed);
    d.setCancelCondition(
        {CALLBACK, [](const CallbackData& data) { return data.td.required_doubles[0] < time::now().asSec(); }});
    addStep(std::move(d));
    // end of skill
}
}  // namespace luhsoccer::skills