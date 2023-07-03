#include "skill_books/bod_skill_book/get_ball.hpp"

#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/features/ball_target_feature.hpp"
#include "local_planner_components/steps/wait_step.hpp"

namespace luhsoccer::skills {

void GetBallBuild::buildImpl(const config_provider::ConfigStore& cs) {
    addStep(DribblerStep(robot_interface::DribblerMode::LOW));

    DriveStep go_to_get_ball;
    go_to_get_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    go_to_get_ball.setAvoidOtherRobots(true);
    go_to_get_ball.setRotationControl(HeadingRotationControl("ball"));
    go_to_get_ball.addFeature(TargetFeature(CircleShape("ball", cs.skills_config.move_to_ball_pre_radius, true)));
    addStep(std::move(go_to_get_ball));

    DriveStep get_ball;
    get_ball.addFeature(BallTargetFeature(CircleShape("ball", cs.skills_config.robot_radius, false)));
    get_ball.setRotationControl(HeadingRotationControl("ball"));
    get_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    get_ball.setMaxVelX(cs.local_planner_components_config.robot_vel_max_x * cs.skills_config.max_vel_drive_dribbling);
    addStep(std::move(get_ball));
    DriveStep drive_in_ball;
    drive_in_ball.addFeature(BallTargetFeature(CircleShape("ball", 0.05, false)));
    drive_in_ball.setRotationControl(HeadingRotationControl("ball"));
    drive_in_ball.setReachCondition(DriveStep::ReachCondition::NEVER);
    drive_in_ball.setCancelCondition({CALLBACK, [](const CallbackData& data) -> bool {
                                          auto start_time =
                                              data.td.getCookie<time::TimePoint>(data.component_uid, "start_time");
                                          if (!start_time.has_value()) {
                                              data.td.setCookie(data.component_uid, "start_time", time::now());
                                              return false;
                                          }
                                          return time::now() - start_time.value() > time::Duration(0.1);
                                      }});
    addStep(std::move(drive_in_ball));

    addStep(DribblerStep(robot_interface::DribblerMode::LOW));
}
}  // namespace luhsoccer::skills