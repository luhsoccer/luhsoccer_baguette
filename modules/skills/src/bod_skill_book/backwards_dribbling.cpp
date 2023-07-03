#include "skill_books/bod_skill_book/backwards_dribbling.hpp"
#include "local_planner_components/features/ball_target_feature.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/shapes/line_shape.hpp"
#include "local_planner_components/steps/condition_step.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/steps/wait_step.hpp"
#include "local_planner_components/features/robot_cf_obstacle.hpp"
#include "local_planner_components/shapes/arc_shape.hpp"
#include "local_planner_components/steps/turn_around_ball_step.hpp"

namespace luhsoccer::skills {

BackwardsDribblingBuild::BackwardsDribblingBuild()
    : SkillBuilder("BackwardsDribbling",  //
                   {},                    //
                   {"TargetPoint"},       //
                   {},                    //
                   {},                    //
                   {"BallPlacement"},     //
                   {}){};

void BackwardsDribblingBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // PARAMS
    ComponentPosition target_point(TD_Pos::POINT, 0);
    constexpr double LET_GO_BALL_TIME = 2.0;

    // stop dribbling condition
    BoolComponentParam drove_too_far(
        CALLBACK, [target_point](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
            auto robot_pos =
                ComponentPosition(td.robot.getFrame()).positionObject(wm, td).getCurrentPosition(wm, "", time::now());
            auto target = target_point.positionObject(wm, td).getCurrentPosition(wm, "", time::now());
            constexpr double MAX_DRIBBLING_DIST = 0.95;
            constexpr double DISTANCE_PUFFER = 0.0;

            if (wm->getLastBallObtainPosition().has_value() && robot_pos.has_value() && target.has_value()) {
                Eigen::Vector2d start_pos = std::get<1>(*wm->getLastBallObtainPosition()).translation();
                Eigen::Vector2d end_pos = (*robot_pos).translation();
                Eigen::Vector2d target_pos = (*target).translation();

                auto max_distance = std::sqrt(std::pow(start_pos.x() - target_pos.x(), 2) +
                                              std::pow(start_pos.y() - target_pos.y(), 2));
                auto distance_travelled =
                    std::sqrt(std::pow(start_pos.x() - end_pos.x(), 2) + std::pow(start_pos.y() - end_pos.y(), 2));
                // once it reached target pose
                if (!td.required_bools[0]) {
                    // exceeded 1m if not in ball placement mode
                    return distance_travelled >= MAX_DRIBBLING_DIST;
                }
                return distance_travelled >= (max_distance - DISTANCE_PUFFER);
            }

            return false;
        });

    // arc position for preposition
    ComponentPosition ball_rotated_to_target(CALLBACK,
                                             [target_point](const std::shared_ptr<const transform::WorldModel>& wm,
                                                            const TaskData& td) -> transform::Position {
                                                 std::optional<Eigen::Affine2d> target_pos =
                                                     target_point.positionObject(wm, td).getCurrentPosition(wm, "ball");
                                                 if (target_pos.has_value()) {
                                                     double target_rot = atan2(target_pos->translation().y(),
                                                                               target_pos->translation().x());
                                                     return {"ball", 0.0, 0.0, target_rot};
                                                 }
                                                 return {"ball"};
                                             });

    // STEPS
    addStep(DribblerStep(robot_interface::DribblerMode::LOW));

    // preposition step
    DriveStep preposition_to_ball;
    preposition_to_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    preposition_to_ball.setAvoidOtherRobots(true);
    preposition_to_ball.setRotationControl(HeadingRotationControl("ball", false, L_PI / 2));
    preposition_to_ball.addFeature(
        TargetFeature(ArcShape(ball_rotated_to_target, cs.skills_config.move_to_ball_pre_radius * 4 / 3,
                               cs.skills_config.kick_ball_through_target_preposition_angle),
                      0.1));
    preposition_to_ball.addFeature(RobotCFObstacle(PointShape("ball"), cs.skills_config.ball_obstacle_weight));
    preposition_to_ball.setCancelCondition(
        {CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> bool {
             return wm->getBallInfo()->isInRobot(td.robot);
         }});
    preposition_to_ball.activateConstraint(DriveStepConstraintNames::BALL_PLACEMENT, false);
    preposition_to_ball.setAvoidDefenseArea(false);
    addStep(std::move(preposition_to_ball));

    // get ball steps
    DriveStep go_to_get_ball;
    go_to_get_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    go_to_get_ball.setAvoidOtherRobots(true);
    go_to_get_ball.setRotationControl(HeadingRotationControl("ball"));
    go_to_get_ball.addFeature(TargetFeature(CircleShape("ball", cs.skills_config.move_to_ball_pre_radius, true)));
    go_to_get_ball.activateConstraint(DriveStepConstraintNames::BALL_PLACEMENT, false);
    go_to_get_ball.setAvoidDefenseArea(false);
    addStep(std::move(go_to_get_ball));

    addStep(DribblerStep(robot_interface::DribblerMode::HIGH));

    DriveStep get_ball;
    get_ball.addFeature(BallTargetFeature(CircleShape("ball", cs.skills_config.robot_radius, false)));
    // get_ball.setRotationControl(HeadingRotationControl("ball"));
    get_ball.setRotationControl(HeadingRotationControl(target_point, true));
    get_ball.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    get_ball.setMaxVelX(cs.local_planner_components_config.robot_vel_max_x * cs.skills_config.max_vel_drive_dribbling);
    get_ball.activateConstraint(DriveStepConstraintNames::BALL_PLACEMENT, false);
    get_ball.setAvoidDefenseArea(false);
    addStep(std::move(get_ball));

    addStep(WaitStep(WAIT_DURATION, cs.skills_config.dribbling_centering_time));

    // dribbling steps
    DriveStep go_to_target;
    go_to_target.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    go_to_target.setRotationControl(HeadingRotationControl(target_point, true));
    go_to_target.addFeature(TargetFeature(PointShape({TD_Pos::POINT, 0})));
    go_to_target.setCancelCondition(drove_too_far);
    go_to_target.setMaxVelX(cs.local_planner_components_config.robot_vel_max_x *
                            cs.skills_config.max_vel_drive_dribbling);
    go_to_target.setMaxVelY(cs.local_planner_components_config.robot_vel_max_y *
                            cs.skills_config.max_vel_drive_dribbling);
    go_to_target.activateConstraint(DriveStepConstraintNames::BALL_PLACEMENT, false);
    go_to_target.setAvoidDefenseArea(false);
    addStep(std::move(go_to_target));

    ConditionStep c({CALLBACK, [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> bool {
                         return td.required_bools[0];
                     }});
    c.addIfStep(DribblerStep(robot_interface::DribblerMode::OFF));
    c.addIfStep(WaitStep(WAIT_DURATION, LET_GO_BALL_TIME));

    DriveStep get_away;
    get_away.addFeature(TargetFeature(CircleShape("ball", cs.skills_config.dist_after_dribbling, false)));
    get_away.setRotationControl(HeadingRotationControl("ball"));
    get_away.setReachCondition(DriveStep::ReachCondition::ONE_OF_TARGETS);
    get_away.setMaxVelX(cs.local_planner_components_config.robot_vel_max_x * cs.skills_config.max_vel_drive_dribbling);
    get_away.activateConstraint(DriveStepConstraintNames::BALL_PLACEMENT, false);
    get_away.setAvoidDefenseArea(false);
    c.addIfStep(std::move(get_away));

    addStep(std::move(c));
}
}  // namespace luhsoccer::skills