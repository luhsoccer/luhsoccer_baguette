#include "skill_books/bod_skill_book/go_to_point_with_heading_original.hpp"
// include components here
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/shapes/point_shape.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"

namespace luhsoccer::skills {

GoToPointWithHeadingOriginalBuild::GoToPointWithHeadingOriginalBuild()
    : SkillBuilder("GoToPointWithHeadingOriginal",          //
                   {},                                      //
                   {"TranslationalPoint", "HeadingPoint"},  //
                   {},                                      //
                   {},                                      //
                   {},                                      //
                   {}){};

void GoToPointWithHeadingOriginalBuild::buildImpl(const config_provider::ConfigStore& /*cs*/) {
    // Add skill definition here. Use addStep to add a step
    addStep(DribblerStep(robot_interface::DribblerMode::OFF));

    DriveStep d;
    d.addFeature(TargetFeature(PointShape({TD_Pos::POINT, 0})));
    d.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    d.setRotationControl(HeadingRotationControl({TD_Pos::POINT, 1}));

    addStep(std::move(d));
    // end of skill
}
}  // namespace luhsoccer::skills