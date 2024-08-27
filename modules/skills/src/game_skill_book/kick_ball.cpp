#include "skill_books/game_skill_book/kick_ball.hpp"
#include "config/skills_config.hpp"
#include "robot_control/components/component_data.hpp"
#include "robot_control/components/features/obstacle_feature.hpp"
#include "robot_control/components/features/target_feature.hpp"
#include "robot_control/components/rotation_controls/heading_rotation_control.hpp"
#include "robot_control/components/shapes/arc_shape.hpp"
#include "robot_control/components/shapes/circle_shape.hpp"
#include "robot_control/components/shapes/line_shape.hpp"
#include "robot_control/components/shapes/point_shape.hpp"
#include "robot_control/components/steps/condition_step.hpp"
#include "robot_control/components/steps/drive_step.hpp"
#include "robot_control/components/steps/kick_step.hpp"
#include "robot_control/components/steps/turn_around_ball_step.hpp"
#include "robot_control/components/steps/wait_step.hpp"
// include components here

namespace luhsoccer::skills {

KickBallBuild::KickBallBuild()
    : SkillBuilder("KickBall",             //
                   {},                     //
                   {"Target Point"},       //
                   {"Kick Velocity"},      //
                   {},                     //
                   {"Free kick", "Chip"},  //
                   {}){};

void KickBallBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step

    addStep(
        ChangeKickerModeStep({CALLBACK, [](const CallbackData& data) -> bool { return data.td.required_bools[1]; }}));

    /// @todo add ball interception
    ConditionStep ball_in_dribbler({CALLBACK, [](const CallbackData& data) -> bool {
                                        auto ball_data = data.wm->getBallInfo();
                                        return ball_data.has_value() && ball_data->isInRobot(data.td.robot);
                                    }});

    DriveStep go_to_ball;
    ComponentPosition ball_rotated_to_target(
        CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> transform::Position {
            std::optional<Eigen::Affine2d> target_pos =
                comp_data.td.required_positions[0].getCurrentPosition(comp_data.wm, "ball", comp_data.time);
            if (target_pos.has_value()) {
                double target_rot = atan2(target_pos->translation().y(), target_pos->translation().x());
                return {"ball", 0.0, 0.0, target_rot + L_PI};
            }
            return {"ball"};
        });

    DoubleComponentParam is_freekick(CALLBACK, [&cs](const ComponentData& comp_data, const ComponentUid&) -> double {
        if (comp_data.td.required_bools[1]) {
            return cs.skills_config.kickball_vel_in_freekick;
        }
        return cs.robot_control_config.robot_max_vel_x;
    });
    DoubleComponentParam adaptive_angle(CALLBACK, [](const ComponentData& comp_data, const ComponentUid&) -> double {
        return comp_data.td.required_bools[0] ? 0.01 : 0.05;
    });

    DoubleComponentParam adaptive_tolerance(
        CALLBACK, [&cs](const ComponentData& comp_data, const ComponentUid& uid) -> double {
            if (comp_data.td.required_bools[0]) return 0.01;

            // check velocity
            transform::Position robot_pos = transform::Position(comp_data.robot.getFrame());
            auto past_position = robot_pos.getCurrentPosition(comp_data.wm, "", time::now() - time::Duration(1.0));
            auto current_position = robot_pos.getCurrentPosition(comp_data.wm, "", time::now());
            if (!past_position.has_value() || !current_position.has_value()) {
                // logger::Logger("Kickball").info("Robot moving, tolerance 0.03, {}");
                return 0.03;
            }
            Eigen::Vector2d diff = past_position->translation() - current_position->translation();
            if (diff.norm() > 0.1) {
                // logger::Logger("Kickball").info("Robot moving, tolerance 0.03, {}", diff.norm());
                return 0.03;
            }
            auto start_time = comp_data.td.getCookie<time::TimePoint>(uid, "start_time_adaptive_tolerance");
            if (!start_time.has_value()) {
                comp_data.td.setCookie(uid, "start_time_adaptive_tolerance", time::now());
                return cs.skills_config.kick_drive_in_ball_tolerance;
            }
            double factor = 1.0;
            if (time::now() - start_time.value() > time::Duration(6.0)) {
                // logger::Logger("Kickball").info("Adaptive tolerance factor 6");
                factor = 6.0;
            } else if (time::now() - start_time.value() > time::Duration(3.0)) {
                factor = 3.0;
                // logger::Logger("Kickball").info("Adaptive tolerance factor 3");
            }

            return factor * cs.skills_config.kick_drive_in_ball_tolerance;
        });
    go_to_ball.setRotationControl(HeadingRotationControl("ball", false, L_PI / 2));

    DoubleComponentParam adaptive_radius(
        CALLBACK, [&cs](const ComponentData& comp_data, const ComponentUid&) -> double {
            double closest_robot_dist = 100.0;
            transform::Position ball_pos("ball");
            for (const auto& robot : comp_data.wm->getVisibleRobots()) {
                if (robot.isAlly()) continue;
                auto ball_in_robot_frame = ball_pos.getCurrentPosition(comp_data.wm, robot.getFrame());
                if (ball_in_robot_frame.has_value() && ball_in_robot_frame->translation().norm() < closest_robot_dist) {
                    closest_robot_dist = ball_in_robot_frame->translation().norm();
                }
            }
            return std::min(cs.skills_config.ball_go_to_ball_radius.val(), closest_robot_dist - 0.1);
        });

    go_to_ball.addFeature(
        TargetFeature(ArcShape(ball_rotated_to_target, adaptive_radius, adaptive_angle), adaptive_tolerance));
    go_to_ball.addFeature(ObstacleFeature(PointShape("ball"), 0.5));
    go_to_ball.setMaxVelX(is_freekick);
    go_to_ball.setMaxVelY(is_freekick);

    ConditionStep far_away_from_ball({CALLBACK, [&cs](const CallbackData& data) {
                                          auto pos = data.wm->getTransform("ball", data.td.robot.getFrame());
                                          if (!pos.has_value()) return false;
                                          return pos->transform.translation().norm() >
                                                 cs.skills_config.ball_go_to_ball_radius;
                                      }});

    DriveStep turn_to_target;
    turn_to_target.setRotationControl(HeadingRotationControl({TD_Pos::POINT, 0}, false, THREE_DEGREE_IN_RADIAN / 2));
    turn_to_target.setReachCondition(DriveStep::ReachCondition::ALL_TARGETS);

    far_away_from_ball.addIfStep(std::move(go_to_ball));

    far_away_from_ball.addElseStep(TurnAroundBallStep({TD_Pos::POINT, 0}));

    ball_in_dribbler.addElseStep(std::move(far_away_from_ball));

    addStep(std::move(ball_in_dribbler));
    addStep(std::move(turn_to_target));

    addStep(KickStep({CALLBACK,
                      [](const ComponentData& comp_data, const ComponentUid&) {
                          return comp_data.td.required_doubles[0] == 0.0 ? MAX_KICK_VELOCITY
                                                                         : comp_data.td.required_doubles[0];
                      }},
                     false, robot_interface::KickExecuteTime::WHEN_BALL_IN_DRIBBLER,
                     {CALLBACK, [](const CallbackData& data) -> bool { return data.td.required_bools[1]; }}));

    DriveStep drive_in_ball;
    drive_in_ball.addFeature(
        BallTargetFeature(CircleShape("ball", 0.03, true), cs.skills_config.kick_drive_in_ball_weight, true));

    DoubleComponentParam adaptive_tolerance_drive_in_ball(
        CALLBACK, [&cs](const ComponentData& data, const ComponentUid& uid) -> double {
            auto start_time = data.td.getCookie<time::TimePoint>(uid, "start_time_adaptive_tolerance");
            if (!start_time.has_value()) {
                data.td.setCookie(uid, "start_time_adaptive_tolerance", time::now());
                return cs.skills_config.kick_drive_in_ball_tolerance;
            }
            double factor = 1.0;
            if (time::now() - start_time.value() > time::Duration(6.0)) {
                factor = 6.0;
            } else if (time::now() - start_time.value() > time::Duration(3.0)) {
                factor = 3.0;
            }

            return factor * cs.skills_config.kick_drive_in_ball_tolerance;
        });
    drive_in_ball.addFeature(
        TargetFeature(LineShape({"ball"}, {TD_Pos::POINT, 0}, -1.0), adaptive_tolerance_drive_in_ball));
    drive_in_ball.setMaxVelX(1.0);
    drive_in_ball.setMaxVelY(1.0);
    drive_in_ball.setAvoidOtherRobots(false);
    drive_in_ball.setRotationControl(HeadingRotationControl({TD_Pos::POINT, 0}));
    drive_in_ball.setReachCondition(DriveStep::ReachCondition::NEVER);
    drive_in_ball.setCancelCondition({CALLBACK, [&cs](const ComponentData& comp_data, const ComponentUid&) -> double {
                                          auto ball_pose = transform::Position("ball").getCurrentPosition(
                                              comp_data.wm, comp_data.td.robot.getFrame(), comp_data.time);
                                          if (!ball_pose.has_value()) return false;
                                          bool ball_shot = ball_pose->translation().norm() >
                                                           cs.skills_config.ball_go_to_ball_radius * 2;
                                          bool ball_beside_robot = std::abs(ball_pose->translation().y()) > 0.06 &&
                                                                   ball_pose->translation().x() < 0.08;
                                          return ball_shot || ball_beside_robot;
                                      }});

    addStep(std::move(drive_in_ball));
    addStep(WaitStep(WAIT_DURATION, 0.2));
    // end of skill
}
}  // namespace luhsoccer::skills