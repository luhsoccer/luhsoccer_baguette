#include "skill_books/bod_skill_book/go_to_point_aligned.hpp"
// include components here
#include "local_planner_components/rotation_controls/rotate_to_move_direction_control.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/steps/condition_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/shapes/point_shape.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"

namespace luhsoccer::skills {

goToPointAlignedBuild::goToPointAlignedBuild()
    : SkillBuilder("goToPointAligned",  //
                   {},                  //
                   {"TargetPose"},      //
                   {},                  //
                   {},                  //
                   {},                  //
                   {}){};

void goToPointAlignedBuild::buildImpl(const config_provider::ConfigStore& /*cs*/) {
    // Add skill definition here. Use addStep to add a step

    BoolComponentParam face_away(CALLBACK, [](const CallbackData& data) -> bool {
        auto target_in_robot_frame =
            data.td.required_positions[0].getCurrentPosition(data.wm, data.td.robot.getFrame());
        return target_in_robot_frame.has_value() && target_in_robot_frame->translation().x() < 0.0;
    });
    DriveStep turn_to_target;
    turn_to_target.addFeature(TargetFeature(PointShape({TD_Pos::EXECUTING_ROBOT, 0}), 1000.0));
    turn_to_target.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    turn_to_target.setRotationControl(HeadingRotationControl({TD_Pos::POINT, 0}, face_away, 2.0));

    DriveStep drive_to_target;
    drive_to_target.addFeature(TargetFeature(PointShape({TD_Pos::POINT, 0}), 1.0));
    drive_to_target.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    drive_to_target.setRotationControl(RotateToMoveDirectionControl(face_away));

    // turn to final heading
    DriveStep turn;
    turn.addFeature(TargetFeature(PointShape({TD_Pos::POINT, 0})));
    turn.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    turn.setRotationControl(HeadingRotationControl(0.0, {TD_Pos::POINT, 0}, THREE_DEGREE_IN_RADIAN, true));

    ConditionStep cond({CALLBACK, [](const CallbackData& data) -> bool {
                            auto goal_transform =
                                data.td.required_positions[0].getCurrentPosition(data.wm, data.td.robot.getFrame());
                            return goal_transform.has_value() && goal_transform->translation().norm() > 2.0;
                        }});
    cond.addIfStep(std::move(turn_to_target));
    cond.addIfStep(std::move(drive_to_target));
    cond.addIfStep(std::move(turn));

    DriveStep drive_direct;
    drive_direct.addFeature(TargetFeature(PointShape({TD_Pos::POINT, 0})));
    drive_direct.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    drive_direct.setRotationControl(HeadingRotationControl(0.0,                     //
                                                           {TD_Pos::POINT, 0},      //
                                                           THREE_DEGREE_IN_RADIAN,  //
                                                           true));

    cond.addElseStep(std::move(drive_direct));
    addStep(std::move(cond));
    // end of skill
}
}  // namespace luhsoccer::skills