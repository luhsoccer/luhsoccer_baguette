#include "skill_books/game_skill_book/dribble.hpp"
#include "ball_steps.hpp"
#include "robot_control/components/component_data.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/point_shape.hpp"
#include "robot_control/components/steps/condition_step.hpp"
#include "robot_control/components/steps/dribbler_step.hpp"
#include "robot_control/components/steps/drive_step.hpp"
#include "robot_control/components/steps/wait_step.hpp"
// include components here

namespace luhsoccer::skills {

DribbleBuild::DribbleBuild()
    : SkillBuilder("Dribble",   //
                   {},          //
                   {"Target"},  //
                   {},          //
                   {},          //
                   {},          //
                   {}){};

void DribbleBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    ComponentPosition target_point(TD_Pos::POINT, 0);

    ConditionStep ball_in_dribbler({CALLBACK, [](const CallbackData& data) -> bool {
                                        auto ball_data = data.wm->getBallInfo();
                                        return ball_data.has_value() && ball_data->isInRobot(data.td.robot);
                                    }});

    ball_in_dribbler.addElseStep(DribblerStep(robot_interface::DribblerMode::LOW));
    ball_in_dribbler.addElseStepPtr(getGetBallStep(cs));

    ball_in_dribbler.addElseStep(WaitStep(WAIT_DURATION, 1.0));

    addStep(std::move(ball_in_dribbler));
    addStep(DribblerStep(robot_interface::DribblerMode::HIGH));

    constexpr double AFTER_HIGH_WAIT_TIME = 0.1;
    addStep(WaitStep(WAIT_DURATION, AFTER_HIGH_WAIT_TIME));

    ComponentPosition valid_target(
        CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> transform::Position {
            auto target_pose = comp_data.td.required_positions[0].getCurrentPosition(comp_data.wm, "", comp_data.time);
            auto last_obtain_position = comp_data.wm->getLastBallObtainPosition();

            if (!target_pose.has_value() || !last_obtain_position.has_value()) {
                return comp_data.td.required_positions[0];
            }

            Eigen::Vector2d diff = target_pose->translation() - last_obtain_position->second.translation();
            constexpr double MAX_DRIBBLING_DIST = 0.95;

            diff *= std::min(1.0, MAX_DRIBBLING_DIST / diff.norm());
            Eigen::Vector2d valid_target = last_obtain_position->second.translation() + diff;

            return {"", valid_target.x(), valid_target.y(), Eigen::Rotation2Dd(target_pose->rotation()).angle()};
        });

    DriveStep go_to_target;
    go_to_target.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    go_to_target.setRotationControl(HeadingRotationControl(0.0, target_point));
    go_to_target.addFeature(TargetFeature(PointShape(valid_target)));
    // go_to_target.setMaxVelX(cs.local_planner_components_config.robot_vel_max_x *
    //                         cs.skills_config.max_vel_drive_dribbling);
    // go_to_target.setMaxVelY(cs.local_planner_components_config.robot_vel_max_y *
    //                         cs.skills_config.max_vel_drive_dribbling);
    addStep(std::move(go_to_target));
    // end of skill
}
}  // namespace luhsoccer::skills