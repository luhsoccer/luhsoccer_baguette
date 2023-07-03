#include "skill_books/bod_skill_book/drive_in_ball.hpp"
// include components here
#include "local_planner_components/features/ball_target_feature.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/shapes/rectangle_shape.hpp"
#include "local_planner_components/steps/turn_around_ball_step.hpp"
#include "local_planner_components/steps/wait_step.hpp"

namespace luhsoccer::skills {

DriveInBallBuild::DriveInBallBuild()
    : SkillBuilder("DriveInBall",     //
                   {},                //
                   {"target"},        //
                   {"max_velocity"},  //
                   {},                //
                   {},                //
                   {}){};

void DriveInBallBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    DriveStep drive_to_ball;
    drive_to_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    drive_to_ball.setRotationControl(HeadingRotationControl("ball"));
    drive_to_ball.addFeature(TargetFeature(CircleShape({"ball"}, {0.2}, {true})));

    TurnAroundBallStep position({TD_Pos::POINT, 0});

    DriveStep drive_in_ball;
    drive_in_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    drive_in_ball.setRotationControl(HeadingRotationControl("ball"));
    drive_in_ball.addFeature(BallTargetFeature(PointShape("ball")));
    drive_in_ball.setMaxVelX({TD, 0});
    drive_in_ball.setMaxVelY({TD, 0});

    WaitStep wait(WAIT_DURATION, {1.0});

    addStep(std::move(drive_to_ball));
    addStep(std::move(position));
    addStep(std::move(drive_in_ball));
    addStep(std::move(wait));
    // end of skill
}
}  // namespace luhsoccer::skills