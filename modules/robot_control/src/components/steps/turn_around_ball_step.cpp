#include "robot_control/components/steps/turn_around_ball_step.hpp"

#include "robot_control/components/component_data.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/features/turn_around_ball_feature.hpp"

namespace luhsoccer::robot_control {

TurnAroundBallStep::TurnAroundBallStep(ComponentPosition align_position) {
    this->setAvoidOtherRobots(false);
    DoubleComponentParam max_rot_vel(CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> double {
        auto ball_info = comp_data.wm->getBallInfo();
        if (!ball_info.has_value() || ball_info->state != transform::BallState::IN_ROBOT ||
            !ball_info->robot.has_value() || ball_info->robot != comp_data.td.robot)
            return robotControlConfig().robot_max_vel_theta;

        return robotControlConfig().turn_around_ball_max_vel_theta;
    });
    this->setRotationControl(HeadingRotationControl(
        {CALLBACK,
         [align_position](const ComponentData& comp_data, const ComponentUid&) -> double {
             auto align_position_vec = align_position.positionObject(comp_data).getCurrentPosition(comp_data.wm, "");
             auto ball_vec = transform::Position("ball").getCurrentPosition(comp_data.wm, "");
             if (!align_position_vec.has_value() || !ball_vec.has_value()) return 0.0;
             Eigen::Vector2d ball_to_align_position = align_position_vec->translation() - ball_vec->translation();
             return std::atan2(ball_to_align_position.y(), ball_to_align_position.x());
         }},
        "", THREE_DEGREE_IN_RADIAN, false, std::nullopt, max_rot_vel));
    this->addFeature(TurnAroundBallFeature(std::move(align_position)));
    this->setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    this->setMaxVelX(robotControlConfig().turn_around_ball_max_vel_x.val());
    this->setMaxVelY(robotControlConfig().turn_around_ball_max_vel_y.val());
    this->setMaxVelT(max_rot_vel);
}

}  // namespace luhsoccer::robot_control
