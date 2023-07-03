#include "skill_books/bod_skill_book/kick_ball_to_target.hpp"
// include components here
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/steps/kick_step.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/steps/turn_around_ball_step.hpp"
#include "local_planner_components/steps/wait_step.hpp"

#include "bod_skill_book/move_to_ball_turn_radius_macro.hpp"
namespace luhsoccer::skills {

KickBallToTargetBuild::KickBallToTargetBuild()
    : SkillBuilder("KickBallToTarget",  //
                   {},                  //
                   {"TargetPoint"},     //
                   {},                  //
                   {},                  //
                   {},                  //
                   {}){};

void KickBallToTargetBuild::buildImpl(const config_provider::ConfigStore& cs) {
    MOVE_TO_BALL_TURN_RADIUS

    addStep(TurnAroundBallStep({TD_Pos::POINT, 0}));

    DoubleComponentParam kick_velocity(
        CALLBACK, [&](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& /*td*/) -> double {
            transform::Position p1("TargetPoint");
            transform::Position p2("ball");

            if (!p1.getCurrentPosition(wm).has_value() || !p2.getCurrentPosition(wm).has_value()) return 4.0;
            auto x_delta = p1.getCurrentPosition(wm)->translation().x() - p2.getCurrentPosition(wm)->translation().x();
            auto y_delta = p1.getCurrentPosition(wm)->translation().y() - p2.getCurrentPosition(wm)->translation().y();
            auto distance = std::sqrt(std::pow(x_delta, 2) - std::pow(y_delta, 2));

            float static_friction = cs.skills_config.static_friction;
            float dynamic_friction = cs.skills_config.dynamic_friction;

            auto kick_velocity = dynamic_friction * distance + static_friction;
            return kick_velocity;
        });

    addStep(KickStep(kick_velocity, false, robot_interface::KickExecuteTime::WHEN_BALL_IN_DRIBBLER));

    DriveStep drive_in_ball;
    drive_in_ball.addFeature(TargetFeature(CircleShape("ball", 0.03, true)));
    drive_in_ball.setRotationControl(HeadingRotationControl({TD_Pos::POINT, 0}));
    drive_in_ball.setReachCondition(DriveStep::ReachCondition::NEVER);
    drive_in_ball.setCancelCondition(
        {CALLBACK, [&cs](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
             auto ball_pose = transform::Position(td.robot.getFrame()).getCurrentPosition(wm, "ball");
             return ball_pose.has_value() &&
                    ball_pose->translation().norm() > cs.skills_config.move_to_ball_pre_radius * 3;
         }});
    addStep(std::move(drive_in_ball));

    // addStep(WaitStep(WAIT_DURATION, 0.2));
    // end of skill
}
}  // namespace luhsoccer::skills