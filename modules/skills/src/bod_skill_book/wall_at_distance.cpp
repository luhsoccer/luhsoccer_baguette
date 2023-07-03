#include "skill_books/bod_skill_book/wall_at_distance.hpp"
#include "local_planner/skills/task.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
// include components here

namespace luhsoccer::skills {

WallAtDistanceBuild::WallAtDistanceBuild()
    : SkillBuilder("WallAtDistance",                                 //
                   {},                                               //
                   {},                                               //
                   {"WallDistance"},                                 //
                   {"DefendingRobotIndex", "TotalDefendingRobots"},  //
                   {},                                               //
                   {}){};

void WallAtDistanceBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // define step
    addStep(DribblerStep(robot_interface::DribblerMode::OFF));

    // add Components
    ComponentPosition ball("ball");
    DoubleComponentParam wall_dist(TD, 0);

    // find needed offset length for wall position distribution
    DoubleComponentParam offset_wall(
        CALLBACK, [&cs](const std::shared_ptr<const transform::WorldModel>& /*wm*/, const TaskData& td) -> double {
            double offset = -(td.required_ints[1] - 1.0) * cs.local_planner_components_config.robot_radius;
            offset += td.required_ints[0] * cs.local_planner_components_config.robot_radius * 2;
            return offset;
        });

    ComponentPosition goal_centre_pose(
        CALLBACK,
        [](const std::shared_ptr<const transform::WorldModel>& /*wm*/, const TaskData& /*td*/) -> transform::Position {
            return {transform::field::GOAL_ALLY_CENTER};
        });

    // find point at distance from goal to ball
    ComponentPosition wall_distance_point(
        CALLBACK,
        [ball, wall_dist](const std::shared_ptr<const transform::WorldModel>& wm,
                          const TaskData& td) -> transform::Position {
            std::optional<Eigen::Affine2d> x_ball =
                ball.positionObject(wm, td).getCurrentPosition(wm, transform::field::GOAL_ALLY_CENTER);
            if (x_ball.has_value()) {
                Eigen::Vector2d ball_vec(x_ball->translation().x(), x_ball->translation().y());
                double ball_distance = ball_vec.norm();
                return {transform::field::GOAL_ALLY_CENTER,
                        Eigen::Translation2d(ball_vec / ball_distance * wall_dist.val(wm, td)) * Eigen::Rotation2Dd(0)};
            }
            return {""};
        });

    // derive orthogonal vector to ball-goal-vector
    auto wall_ortogonal_vec = [ball, wall_distance_point](const std::shared_ptr<const transform::WorldModel>& wm,
                                                          const TaskData& td) -> std::optional<Eigen::Vector2d> {
        std::optional<Eigen::Affine2d> x_ball =
            ball.positionObject(wm, td).getCurrentPosition(wm, transform::field::GOAL_ALLY_CENTER);
        if (x_ball.has_value()) {
            Eigen::Vector2d ball_vec(x_ball->translation().x(), x_ball->translation().y());
            return Eigen::Vector2d(1.0, (-ball_vec.x() / ball_vec.y()));
        }
        return std::nullopt;
    };

    // offset goal centre for wall positioning
    ComponentPosition goal_centre_pose_offset(
        CALLBACK,
        [wall_ortogonal_vec, offset_wall, goal_centre_pose](const std::shared_ptr<const transform::WorldModel>& wm,
                                                            const TaskData& td) -> transform::Position {
            std::optional<Eigen::Affine2d> x_goal_centre_pose =
                goal_centre_pose.positionObject(wm, td).getCurrentPosition(wm);
            auto ort_vec = wall_ortogonal_vec(wm, td);
            if (x_goal_centre_pose.has_value() && ort_vec.has_value()) {
                Eigen::Vector2d offset_vec = ort_vec.value() / ort_vec.value().norm() * offset_wall.val(wm, td);
                return {"",
                        Eigen::Translation2d(x_goal_centre_pose->translation() + offset_vec) * Eigen::Rotation2Dd(0)};
            }
            return {""};
        });

    // offset ball pose for wall positioning
    ComponentPosition ball_offset(
        CALLBACK,
        [wall_ortogonal_vec, offset_wall, ball](const std::shared_ptr<const transform::WorldModel>& wm,
                                                const TaskData& td) -> transform::Position {
            std::optional<Eigen::Affine2d> x_ball = ball.positionObject(wm, td).getCurrentPosition(wm);
            auto ort_vec = wall_ortogonal_vec(wm, td);
            if (x_ball.has_value() && ort_vec.has_value()) {
                Eigen::Vector2d offset_vec = ort_vec.value() / ort_vec.value().norm() * offset_wall.val(wm, td);
                return {"", Eigen::Translation2d(x_ball->translation() + offset_vec) * Eigen::Rotation2Dd(0)};
            }
            return {""};
        });

    // define drivestep
    DriveStep wall;
    wall.addFeature(TargetFeature(LineShape(ball_offset, goal_centre_pose_offset)));

    // normalize orthogonal vector
    wall.addFeature(TargetFeature(LineShape(
        wall_distance_point,
        {CALLBACK,
         [wall_distance_point, wall_ortogonal_vec](const std::shared_ptr<const transform::WorldModel>& wm,
                                                   const TaskData& td) -> transform::Position {
             auto v_ort_wall = wall_ortogonal_vec(wm, td);
             std::optional<Eigen::Affine2d> x_wall = wall_distance_point.positionObject(wm, td).getCurrentPosition(wm);
             if (x_wall.has_value() && v_ort_wall.has_value()) {
                 Eigen::Vector2d ort_pose(v_ort_wall.value() / v_ort_wall.value().norm() + x_wall->translation());
                 return {"", Eigen::Translation2d(ort_pose) * Eigen::Rotation2Dd(0)};
             }
             return {""};
         }},
        -1.0, 0.0)));

    wall.setReachCondition(DriveStep::ReachCondition::NEVER);
    wall.setRotationControl(HeadingRotationControl(ball));
    addStep(std::move(wall));
}

}  // namespace luhsoccer::skills