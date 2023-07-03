#include "skill_books/bod_skill_book/drive_to_line.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/steps/drive_step.hpp"
// include components here

namespace luhsoccer::skills {

DriveToLineBuild::DriveToLineBuild()
    : SkillBuilder("DriveToLine",      //
                   {},                 //
                   {"point_on_line"},  //
                   {"angle_line"},     //
                   {},                 //
                   {},                 //
                   {}){};

void DriveToLineBuild::buildImpl(const config_provider::ConfigStore& /*cs*/) {
    // Add skill definition here. Use addStep to add a step
    DriveStep drive_to_line;

    drive_to_line.setRotationControl(HeadingRotationControl("ball"));
    drive_to_line.addFeature(TargetFeature(LineShape({TD_Pos::POINT, 0}, {TD, 0}, {20.0}, {-20.0})));
    drive_to_line.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);

    addStep(std::move(drive_to_line));
    // end of skill
}
}  // namespace luhsoccer::skills