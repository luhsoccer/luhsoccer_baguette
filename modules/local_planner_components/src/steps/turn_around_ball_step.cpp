#include <utility>

#include "local_planner/skills/skill_util.hpp"
#include "local_planner_components/steps/turn_around_ball_step.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/features/turn_around_ball_feature.hpp"
namespace luhsoccer::local_planner {

TurnAroundBallStep::TurnAroundBallStep(ComponentPosition align_position) {
    this->setAvoidOtherRobots(false);
    DoubleComponentParam max_rot_vel(
        CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            auto ball_info = wm->getBallInfo();
            if (!ball_info.has_value() || ball_info->state != transform::BallState::IN_ROBOT ||
                !ball_info->robot.has_value() || ball_info->robot != td.robot)
                return localPlannerConfig().robot_vel_max_theta;

            return config_provider::ConfigProvider::getConfigStore()
                .skills_config.robot_vel_max_theta_turn_around_ball.val();
        });
    this->setRotationControl(HeadingRotationControl(
        {CALLBACK,
         [align_position](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
             auto align_position_vec = align_position.positionObject(wm, td).getCurrentPosition(wm, "");
             auto ball_vec = transform::Position("ball").getCurrentPosition(wm, "");
             if (!align_position_vec.has_value() || !ball_vec.has_value()) return 0.0;
             Eigen::Vector2d ball_to_align_position = align_position_vec->translation() - ball_vec->translation();
             return std::atan2(ball_to_align_position.y(), ball_to_align_position.x());
         }},
        "", THREE_DEGREE_IN_RADIAN, false, std::nullopt, std::nullopt, max_rot_vel));
    this->addFeature(TurnAroundBallFeature(std::move(align_position)));
    this->setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    this->setMaxVelX(localPlannerConfig().feature_turn_around_ball_max_vel_x.val());
    this->setMaxVelY(localPlannerConfig().feature_turn_around_ball_max_vel_y.val());
    this->setMaxVelT(max_rot_vel);
}

}  // namespace luhsoccer::local_planner
