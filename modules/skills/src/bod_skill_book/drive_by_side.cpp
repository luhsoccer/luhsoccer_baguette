#include "skill_books/bod_skill_book/drive_by_side.hpp"
// include components here
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/shapes/point_shape.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
namespace luhsoccer::skills {

DriveBySideBuild::DriveBySideBuild()
    : SkillBuilder("DriveBySide",    //
                   {"TargetRobot"},  //
                   {},               //
                   {},               //
                   {},               //
                   {},               //
                   {}){};

void DriveBySideBuild::buildImpl(const config_provider::ConfigStore& /*cs*/) {
    // Add skill definition here. Use addStep to add a step
    DriveStep d1;
    d1.addFeature(TargetFeature(
        PointShape({CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& /*wm*/, const TaskData& td) {
                        return transform::Position(td.related_robots[0].getFrame(), 0.0, 1.7, 0.0);
                    }})));
    d1.setReachCondition(DriveStep::ReachCondition::NEVER);
    d1.setRotationControl(HeadingRotationControl({TD_Pos::ROBOT, 0}));

    addStep(std::move(d1));
    // end of skill
}
}  // namespace luhsoccer::skills