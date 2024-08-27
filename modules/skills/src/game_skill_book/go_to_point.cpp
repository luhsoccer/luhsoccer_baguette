#include "skill_books/game_skill_book/go_to_point.hpp"
// include components here
#include "robot_control/components/steps/dribbler_step.hpp"
#include "robot_control/components/steps/drive_step.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/point_shape.hpp"

namespace luhsoccer::skills {

GoToPointBuild::GoToPointBuild()
    : SkillBuilder("GoToPoint",      //
                   {},               //
                   {"TargetPoint"},  //
                   {},               //
                   {},               //
                   {},               //
                   {}){};

void GoToPointBuild::buildImpl(const config_provider::ConfigStore& /*cs*/) {
    // Add skill definition here. Use addStep to add a step
    // addStep(DribblerStep(robot_interface::DribblerMode::OFF));

    DriveStep d;
    d.addFeature(TargetFeature(PointShape({TD_Pos::POINT, 0}), DEFAULT_TRANSLATIONAL_TOLERANCE, 1.0, false, true));
    d.setRotationControl(HeadingRotationControl(0.0, {TD_Pos::POINT, 0}, THREE_DEGREE_IN_RADIAN, true));

    addStep(std::move(d));
    // end of skill
}
}  // namespace luhsoccer::skills