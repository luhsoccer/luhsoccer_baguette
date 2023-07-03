#include "skill_books/bod_skill_book/drive_to_line_segment.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
// include components here

namespace luhsoccer::skills {

DriveToLineSegmentBuild::DriveToLineSegmentBuild()
    : SkillBuilder("DriveToLineSegment",  //
                   {},                    //
                   {"p1", "p2"},          //
                   {},                    //
                   {},                    //
                   {},                    //
                   {}){};

void DriveToLineSegmentBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    addStep(DribblerStep(robot_interface::DribblerMode::LOW));

    DriveStep drive_to_line_segment;
    drive_to_line_segment.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    drive_to_line_segment.setRotationControl(HeadingRotationControl({TD_Pos::EXECUTING_ROBOT, 0}));
    drive_to_line_segment.setAvoidOtherRobots(false);
    drive_to_line_segment.addFeature(TargetFeature(LineShape({TD_Pos::POINT, 0}, {TD_Pos::POINT, 1})));
    addStep(std::move(drive_to_line_segment));
    // end of skill
}
}  // namespace luhsoccer::skills