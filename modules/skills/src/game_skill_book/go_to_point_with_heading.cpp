#include "skill_books/game_skill_book/go_to_point_with_heading.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/point_shape.hpp"
#include "robot_control/components/steps/drive_step.hpp"
// include components here

namespace luhsoccer::skills {

GoToPointWithHeadingBuild::GoToPointWithHeadingBuild()
    : SkillBuilder("GoToPointWithHeading",                //
                   {},                                    //
                   {"TranslationPoint", "HeadingPoint"},  //
                   {},                                    //
                   {},                                    //
                   {},                                    //
                   {}){};

void GoToPointWithHeadingBuild::buildImpl(const config_provider::ConfigStore& /*cs*/) {
    // Add skill definition here. Use addStep to add a step
    DriveStep d;
    TargetFeature t(PointShape({TD_Pos::POINT, 0}));
    t.setVelocityZeroForReach(true);
    d.addFeature(std::move(t));
    d.setRotationControl(HeadingRotationControl({TD_Pos::POINT, 1}, false, THREE_DEGREE_IN_RADIAN, true));

    addStep(std::move(d));
    // end of skill
}
}  // namespace luhsoccer::skills