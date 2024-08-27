#include "skill_books/bod_skill_book/block_enemy_line_of_sight.hpp"

// include components here
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "config/skills_config.hpp"

namespace luhsoccer::skills {

BlockEnemyLineOfSightBuild::BlockEnemyLineOfSightBuild()
    : SkillBuilder("BlockEnemyLineOfSight",  //
                   {"EnemyBallCarrier"},     //
                   {},                       //
                   {},                       //
                   {},                       //
                   {},                       //
                   {}){};

void BlockEnemyLineOfSightBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    addStep(DribblerStep(robot_interface::DribblerMode::OFF));
    DriveStep drive_to_line_of_sight;

    drive_to_line_of_sight.setReachCondition(DriveStep::ReachCondition::NEVER);
    drive_to_line_of_sight.setAvoidOtherRobots(true);
    drive_to_line_of_sight.setRotationControl(HeadingRotationControl("ball"));

    auto fieldline_puffer = [&cs](const std::shared_ptr<const transform::WorldModel>& wm,
                                  const TaskData& td) -> double {
        double puffer = cs.skills_config.missing_field_edge + cs.local_planner_components_config.robot_radius +
                        cs.skills_config.block_enemy_line_of_sight_radius;
        return puffer;
    };

    DoubleComponentParam line_cutoff_start(
        CALLBACK, [&cs](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            return -cs.skills_config.block_enemy_line_of_sight_radius * 1.1;
        });

    DoubleComponentParam line_cutoff_end(
        CALLBACK, [&cs](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            auto ball = ComponentPosition("ball").positionObject(wm, td).getCurrentPosition(wm);
            auto enemy = ComponentPosition({TD_Pos::ROBOT, 0}).positionObject(wm, td).getCurrentPosition(wm);
            if (ball.has_value() && enemy.has_value())
                return (ball->translation() - enemy->translation()).norm() +
                       (cs.skills_config.block_enemy_line_of_sight_radius * 0.9);
            return cs.skills_config.block_enemy_line_of_sight_radius * 0.9;
        });

    ComponentPosition lineofsight_object(
        CALLBACK,
        [&cs, fieldline_puffer](const std::shared_ptr<const transform::WorldModel>& wm,
                                const TaskData& td) -> transform::Position {
            auto ball = ComponentPosition("ball").positionObject(wm, td).getCurrentPosition(wm);
            auto enemy = ComponentPosition({TD_Pos::ROBOT, 0}).positionObject(wm, td).getCurrentPosition(wm);
            ComponentPosition field_corner_ally_left = {transform::field::CORNER_ALLY_LEFT};
            auto corner_ally_left = field_corner_ally_left.positionObject(wm, td).getCurrentPosition(wm);
            ComponentPosition field_corner_enemy_left = {transform::field::CORNER_ENEMY_LEFT};
            auto corner_enemy_left = field_corner_enemy_left.positionObject(wm, td).getCurrentPosition(wm);
            auto ball2enemy = ComponentPosition({TD_Pos::ROBOT, 0})
                                  .positionObject(wm, td)
                                  .getCurrentPosition(wm, "ball", time::TimePoint(0));
            auto goal2ball = ComponentPosition("ball").positionObject(wm, td).getCurrentPosition(
                wm, transform::field::GOAL_ALLY_CENTER, time::TimePoint(0));

            // check all edge cases if LoS is outside field
            if (ball.has_value() && enemy.has_value() && corner_ally_left.has_value() &&
                corner_enemy_left.has_value() && ball2enemy.has_value() && goal2ball.has_value()) {
                // check LoS if too close to our goalline
                if (abs(ball->translation().x()) >
                    abs((abs(corner_ally_left->translation().x()) - fieldline_puffer(wm, td)))) {
                    // calculate allowed min angle
                    double distance_h = abs(abs(ball->translation().x()) + fieldline_puffer(wm, td) -
                                            abs(corner_ally_left->translation().x()));
                    double distance_s = 2 * sqrt(2 * cs.skills_config.block_enemy_line_of_sight_radius * distance_h -
                                                 pow(distance_h, 2));
                    double angle_to_wall = 4 * atan2(2 * distance_h, distance_s) / 2;

                    // calculate current angle
                    double angle_to_ball = atan2(ball2enemy->translation().y(), ball2enemy->translation().x());

                    // check if LoS faces out of field
                    if ((abs(angle_to_ball) < abs(angle_to_wall) && ball->translation().x() < 0.0) ||
                        (abs(angle_to_ball) > abs(L_PI - angle_to_wall) && ball->translation().x() > 0.0)) {
                        // return LoS towards goal
                        Eigen::Translation2d goal2ball_vec_norm(goal2ball->translation() /
                                                                goal2ball->translation().norm() *
                                                                ball2enemy->translation().norm());
                        return {"ball", goal2ball_vec_norm * Eigen::Rotation2Dd(0)};
                    }
                }

                // check LoS if too close to goalline
                if (abs(ball->translation().y()) >
                    abs((abs(corner_ally_left->translation().y()) - fieldline_puffer(wm, td)))) {
                    // calculate allowed min angle
                    double distance_h = abs(abs(ball->translation().y()) + fieldline_puffer(wm, td) -
                                            abs(corner_ally_left->translation().y()));
                    double distance_s = 2 * sqrt(2 * cs.skills_config.block_enemy_line_of_sight_radius * distance_h -
                                                 pow(distance_h, 2));
                    double angle_to_wall = 4 * atan2(2 * distance_h, distance_s) / 2;

                    // calculate current angle
                    double angle_to_ball = atan2(ball2enemy->translation().x(), ball2enemy->translation().y());

                    // check if LoS faces out of field and is at corner
                    if (((abs(angle_to_ball) < abs(angle_to_wall) && ball->translation().y() < 0.0) ||
                         (abs(angle_to_ball) > abs(L_PI - angle_to_wall) && ball->translation().y() > 0.0)) &&
                        (abs(ball->translation().x()) >
                         abs((abs(corner_ally_left->translation().x()) - fieldline_puffer(wm, td))))) {
                        // return LoS towards goal
                        Eigen::Translation2d goal2ball_vec_norm(goal2ball->translation() /
                                                                goal2ball->translation().norm() *
                                                                ball2enemy->translation().norm());
                        return {"ball", goal2ball_vec_norm * Eigen::Rotation2Dd(0)};
                    }
                    // check if LoS faces out of right side of field
                    if (abs(angle_to_ball) < abs(angle_to_wall) && ball->translation().y() < 0.0) {
                        // rotate LoS vector to face onto field edge
                        double alpha = 0;
                        if (angle_to_ball > 0) {
                            alpha = angle_to_ball - angle_to_wall;
                        } else {
                            alpha = angle_to_wall - abs(angle_to_ball);
                        }
                        Eigen::Translation2d ball2enemy_vec_tf(ball2enemy->translation());
                        ball2enemy_vec_tf = {ball2enemy_vec_tf.x() * cos(alpha) - ball2enemy_vec_tf.y() * sin(alpha),
                                             ball2enemy_vec_tf.x() * sin(alpha) + ball2enemy_vec_tf.y() * cos(alpha)};
                        // return LoS projected onto field
                        return {"ball", ball2enemy_vec_tf * Eigen::Rotation2Dd(0)};
                    }
                    // check if LoS faces out of left side of field
                    if (abs(angle_to_ball) > abs(L_PI - angle_to_wall) && ball->translation().y() > 0.0) {
                        // rotate LoS vector to face onto field edge
                        double alpha = 0;
                        if (angle_to_ball > 0) {
                            alpha = angle_to_wall + angle_to_ball - L_PI;
                        } else {
                            alpha = L_PI - angle_to_wall - abs(angle_to_ball);
                        }
                        Eigen::Translation2d ball2enemy_vec_tf(ball2enemy->translation());
                        ball2enemy_vec_tf = {ball2enemy_vec_tf.x() * cos(alpha) - ball2enemy_vec_tf.y() * sin(alpha),
                                             ball2enemy_vec_tf.x() * sin(alpha) + ball2enemy_vec_tf.y() * cos(alpha)};
                        // return LoS projected onto field
                        return {"ball", ball2enemy_vec_tf * Eigen::Rotation2Dd(0)};
                    }
                }
            }
            return ComponentPosition(TD_Pos::ROBOT, 0).positionObject(wm, td);
        });

    drive_to_line_of_sight.addFeature(
        TargetFeature(LineShape("ball", lineofsight_object, line_cutoff_start, line_cutoff_end),
                      cs.skills_config.translational_tolerance, 1.0));

    drive_to_line_of_sight.addFeature(
        TargetFeature(CircleShape("ball", cs.skills_config.block_enemy_line_of_sight_radius, false),
                      cs.skills_config.translational_tolerance, cs.skills_config.circle_weight));

    addStep(std::move(drive_to_line_of_sight));
    // end of skill
}
}  // namespace luhsoccer::skills