#include "skill_books/bod_skill_book/defend_goalline.hpp"
// include components here
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/features/target_feature.hpp"

namespace luhsoccer::skills {

DefendGoallineBuild::DefendGoallineBuild()
    : SkillBuilder("DefendGoalline",      //
                   {"EnemyBallCarrier"},  //
                   {},                    //
                   {},                    //
                   {},                    //
                   {},                    //
                   {}){};

void DefendGoallineBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // add Dribblerstep OFF while ball not shot
    addStep(DribblerStep(robot_interface::DribblerMode::OFF));
    // add Drivestep mark enemy / lineofsight while ball not shot
    DriveStep goalline_standby;

    // params
    ComponentPosition ball("ball");
    ComponentPosition ball_carrier(TD_Pos::ROBOT, 0);
    ComponentPosition goal_top(transform::field::GOAL_ALLY_LEFT);
    ComponentPosition goal_bottom(transform::field::GOAL_ALLY_RIGHT);
    double vel_threshold = cs.skills_config.ball_filter_vel_threshold;  // threshold to filter ball velocity variance

    // general settings
    goalline_standby.setReachCondition(DriveStep::ReachCondition::NEVER);
    goalline_standby.setAvoidOtherRobots(false);
    goalline_standby.setAvoidDefenseArea(false);

    // rotation control
    goalline_standby.setRotationControl(HeadingRotationControl("ball"));

    // parameter goal_width
    auto get_goal_width = [goal_top, goal_bottom](const std::shared_ptr<const transform::WorldModel>& wm,
                                                  const TaskData& td) -> double {
        std::optional<Eigen::Affine2d> x_goal_top = goal_top.positionObject(wm, td).getCurrentPosition(wm);
        std::optional<Eigen::Affine2d> x_goal_bottom = goal_bottom.positionObject(wm, td).getCurrentPosition(wm);
        if (x_goal_top.has_value() && x_goal_bottom.has_value()) {
            double goal_width = abs(x_goal_top->translation().y()) + abs(x_goal_bottom->translation().y());
            return goal_width;
        }
        return 0.0;
    };

    // line of sight projection ball / enemy
    auto v_ball_enemy_projection = [ball_carrier, ball](const std::shared_ptr<const transform::WorldModel>& wm,
                                                        const TaskData& td) -> std::optional<Eigen::Vector2d> {
        // project line between ball and enemy onto goal
        std::optional<Eigen::Affine2d> x_carrier =
            ball_carrier.positionObject(wm, td).getCurrentPosition(wm, transform::field::GOAL_ALLY_CENTER);
        std::optional<Eigen::Affine2d> x_ball =
            ball.positionObject(wm, td).getCurrentPosition(wm, transform::field::GOAL_ALLY_CENTER);
        if (x_carrier.has_value() && x_ball.has_value()) {
            Eigen::Vector2d v_carrier_ball = {x_ball->translation().x() - x_carrier->translation().x(),
                                              x_ball->translation().y() - x_carrier->translation().y()};
            if (v_carrier_ball.x() != 0) {
                double y_onto_goal = v_carrier_ball.y() / v_carrier_ball.x() * -x_carrier->translation().x() +
                                     x_carrier->translation().y();
                return Eigen::Vector2d({0.0, y_onto_goal});
            }
        }
        return Eigen::Vector2d({100.0, 100.0});
    };

    // project ball velocity onto goal
    auto v_ball_projection = [ball, vel_threshold](const std::shared_ptr<const transform::WorldModel>& wm,
                                                   const TaskData& td) -> std::optional<Eigen::Vector2d> {
        // project ball direction onto goal
        std::optional<Eigen::Affine2d> x_ball_now =
            ball.positionObject(wm, td).getCurrentPosition(wm, transform::field::GOAL_ALLY_CENTER);
        std::optional<Eigen::Vector3d> v_ball_vel =
            ball.positionObject(wm, td).getVelocity(wm, transform::field::GOAL_ALLY_CENTER);
        if (x_ball_now.has_value() && v_ball_vel.has_value()) {
            if (v_ball_vel->x() < 0.0 && v_ball_vel->norm() > vel_threshold) {
                double y_onto_goal = (v_ball_vel->y() / v_ball_vel->x()) * -x_ball_now->translation().x() +
                                     x_ball_now->translation().y();
                return Eigen::Vector2d({0.0, y_onto_goal});
            }
        }
        return Eigen::Vector2d(100.0, 100.0);
    };

    // project line of sight of enemy onto goal
    auto v_line_of_sight = [ball_carrier](const std::shared_ptr<const transform::WorldModel>& wm,
                                          const TaskData& td) -> std::optional<Eigen::Vector2d> {
        std::optional<Eigen::Affine2d> x_ball_carrier =
            ball_carrier.positionObject(wm, td).getCurrentPosition(wm, transform::field::GOAL_ALLY_CENTER);
        if (x_ball_carrier.has_value()) {
            // generate LineofSight
            Eigen::Rotation2Dd rotation_matrix(x_ball_carrier->rotation());
            double carrier_rot = rotation_matrix.angle();
            if (carrier_rot > L_PI / 2 || carrier_rot < -L_PI / 2) {
                double y_facing_onto_goal =
                    x_ball_carrier->translation().y() + tan(L_PI - carrier_rot) * x_ball_carrier->translation().x();
                return Eigen::Vector2d(0.0, y_facing_onto_goal);
            }
        }
        return Eigen::Vector2d(100.0, 100.0);
    };

    // position in goal relative to ball position
    auto v_ball_position = [ball, get_goal_width](const std::shared_ptr<const transform::WorldModel>& wm,
                                                  const TaskData& td) -> std::optional<Eigen::Vector2d> {
        std::optional<Eigen::Affine2d> x_ball_now =
            ball.positionObject(wm, td).getCurrentPosition(wm, transform::field::GOAL_ALLY_CENTER);
        auto goal_width = get_goal_width(wm, td);
        if (x_ball_now.has_value() && goal_width != 0.0) {
            double y_rel_goal_pose =
                (goal_width / 2) / (L_PI / 2) * atan2(x_ball_now->translation().y(), x_ball_now->translation().x());
            return Eigen::Vector2d(0.0, y_rel_goal_pose);
        }
        return Eigen::Vector2d(0.0, 0.0);
    };

    goalline_standby.addFeature(TargetFeature(LineShape(goal_top, goal_bottom)));
    goalline_standby.addFeature(TargetFeature(
        LineShape(ball, {CALLBACK,
                         [v_ball_projection, get_goal_width, v_ball_enemy_projection, v_ball_position](
                             const std::shared_ptr<const transform::WorldModel>& wm,
                             const TaskData& td) -> transform::Position {
                             // check ball projection
                             auto v_ball_onto_goal = v_ball_projection(wm, td);
                             auto goal_width = get_goal_width(wm, td);
                             if (v_ball_onto_goal->y() < goal_width / 2 && v_ball_onto_goal->y() > -goal_width / 2) {
                                 return {transform::field::GOAL_ALLY_CENTER,
                                         Eigen::Translation2d(v_ball_onto_goal.value()) * Eigen::Rotation2Dd(0)};
                             }

                             // check line of sight (if enemy < min distance away from ball)
                             auto v_sight_onto_goal = v_ball_enemy_projection(wm, td);
                             if (v_sight_onto_goal->y() < goal_width / 2 && v_sight_onto_goal->y() > -goal_width / 2) {
                                 return {transform::field::GOAL_ALLY_CENTER,
                                         Eigen::Translation2d(v_sight_onto_goal.value()) * Eigen::Rotation2Dd(0)};
                             }

                             // else mark ball
                             auto ball_position_onto_goal = v_ball_position(wm, td);
                             return {transform::field::GOAL_ALLY_CENTER,
                                     Eigen::Translation2d(ball_position_onto_goal.value()) * Eigen::Rotation2Dd(0)};
                         }}),
        cs.skills_config.translational_tolerance, 1.0, true));

    addStep(std::move(goalline_standby));
}
}  // namespace luhsoccer::skills