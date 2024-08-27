#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

struct SkillsConfig : public Config {
    SkillsConfig() : Config("skills.toml", datatypes::ConfigType::SHARED) {}

    // Marking
    DoubleParam marking_distance_weight = createDoubleParam(
        "marking_distance_weight", "Weight of the distance when marking enemies.", "Marking", 0.9, 0.1, 1);

    DoubleParam marking_enemy_radius = createDoubleParam(
        "marking_enemy_radius", "Constant radius distance to enemy while marking it [in m]", "Marking", 0.5, 0.18, 3.0);

    DoubleParam marking_wall_defense_area_margin = createDoubleParam(
        "marking_wall_defense_area_margin", "Buffer distance for robot from defense area [robot radius + 0.01m]",
        "Marking", 0.2, 0.09, 0.5);

    // Goalie
    DoubleParam goalie_defend_on_circle_offset =
        createDoubleParam("goalie_defend_on_circle_offset", "Offset of the circle", "Goalie", 1.0, 0.0, 4.0);

    DoubleParam goalie_defend_on_circle_radius =
        createDoubleParam("goalie_defend_on_circle_radius", "Radius of the circle", "Goalie", 1.44, 0.0, 4.0);

    DoubleParam goalie_defend_on_circle_weight =
        createDoubleParam("goalie_defend_on_circle_weight", "Weight of the circle", "Goalie", 0.4, 0.0, 1.0);

    DoubleParam goalie_defend_on_circle_cutoff =
        createDoubleParam("goalie_defend_on_circle_cutoff", "Line cutoff at the goal side", "Goalie", 0.0, -2.0, 2.0);

    DoubleParam goalie_defend_goal_margin = createDoubleParam(
        "goalie_defend_goal_margin", "Margin for goal size to when defending los", "Goalie", 0.15, 0.0, 1.0);

    DoubleParam goalie_reflex_angle_scaling =
        createDoubleParam("goalie_reflex_angle_scaling", "Scaling of the incoming angle added to the outgoing angle",
                          "Goalie", 0.5, 0.0, 1.0);

    // Ball
    DoubleParam ball_go_to_ball_radius = createDoubleParam(
        "ball_go_to_ball_radius", "Radius to ball before retrieving it [in m]", "Ball", 0.25, 0.0, 1.0);

    DoubleParam ball_get_ball_radius =
        createDoubleParam("ball_get_ball_radius", "Radius to ball when in dribbler [in m]", "Ball", 0.07, 0.0, 0.1);

    DoubleParam ball_get_ball_k_p_scaling =
        createDoubleParam("ball_get_ball_k", "Target k_p scaling when get ball", "Ball", 0.8, 0.0, 1.0);

    DoubleParam ball_distance_for_pre_drive = createDoubleParam(
        "ball_distance_for_pre_drive", "Minimal distance to make a pre drive when getting the ball", "Ball", 100.0);

    DoubleParam ball_in_shot_vel = createDoubleParam(
        "ball_in_shot_vel", "Min velocity of ball to be considered in shot [m/s]", "Ball", 0.5, 0.0, 6.5);

    DoubleParam intercept_ball_margin = createDoubleParam(
        "intercept_ball_margin",
        "margin from the field lines where the robot should no longer move backwards to absorb the ball", "Ball", 0.2,
        0.0, 1.5);

    DoubleParam intercept_get_ball_distance =
        createDoubleParam("intercept_get_ball_distance",
                          "Maximum distance to ball for the robot to start getting the ball", "Ball", 1.5, 0.0, 5.0);
    DoubleParam intercept_drive_k =
        createDoubleParam("intercept_drive_k", "Multiplier of kp value for intercept", "Ball", 1.1, 0.0, 5.0);
    DoubleParam reflex_kick_offset = createDoubleParam(
        "reflex_kick_offset", "Offset due to kicker position for reflex kick", "Ball", 0.09, 0.0, 0.2);

    DoubleParam kick_drive_in_ball_tolerance =
        createDoubleParam("kick_drive_in_ball_tolerance", "Translational tolerance of line while driving in ball",
                          "Ball", 0.03, 0.0, 0.1);

    DoubleParam kick_drive_in_ball_weight =
        createDoubleParam("kick_drive_in_ball_weight", "Weight of line while driving in ball", "Ball", 0.5, 0.0, 1.0);

    // Octoskill
    DoubleParam octo_skill_border_margin = createDoubleParam(
        "octo_skill_border_margin", "Distance to the edge of the field, from where the skill is canceled", "OctoSkill",
        0.3, 0.1, 2.0);
    DoubleParam octo_skill_max_vel =
        createDoubleParam("octo_skill_max_vel", "Max speed of the octopus", "OctoSkill", 2.0, 0.1, 10.0);

    // PlaceBall
    DoubleParam placeball_skill_ball_too_close_margin =
        createDoubleParam("placeball_skill_ball_too_close_margin",
                          "Distance from the wall where the robot is supposed to preposition the ball first in order "
                          "to make BallPlacement forwards possible",
                          "PlaceBall", 0.4, 0.0, 1.0);

    // KickBall
    DoubleParam kickball_vel_in_freekick = createDoubleParam(
        "placeball_vel_in_freekick", "Velocity when free kick bool is set", "KickBall", 0.5, 0.0, 2.0);

    // params for genereal skills
    // DoubleParam missing_field_edge =
    //     createDoubleParam("puffer_to_fieldline",
    //                       "in the testing area the space outside of fieldlines is too small. This parameter is the "
    //                       "missing distance AND NEEDS TO BE SET TO ZERO FOR THE TOURNAMENT",
    //                       "GeneralSkills", 0.0);

    // DoubleParam translational_tolerance = createDoubleParam(
    //     "translational_tolerance", "Translational tolerance of target feature [im m]", "GeneralSkills", 0.03);

    // DoubleParam move_to_ball_pre_radius = createDoubleParam(
    //     "move_to_ball_pre_radius", "Radius to ball before retrieving it [in m]", "GeneralSkills", 0.25, 0.0, 1.0);

    // DoubleParam ball_filter_vel_threshold = createDoubleParam(
    //     "ball_filter_vel_threshold", "Threshold of ball velocity vector length, in order to filter out ball
    //     variance", "GeneralSkills", 0.15, 0.0, 1.0);
    // DoubleParam robot_vel_max_theta_turn_around_ball =
    //     createDoubleParam("robot_vel_max_theta_turn_around_ball",
    //                       "Velocity for TurnAroundBall Feature when the ball is in dribbler", "robot", 4.0,
    //                       0.1, 8.0);

    // DoubleParam ball_obstacle_weight = createDoubleParam(
    //     "ball_obstacle_weight", "Weight of ball obstacle feature when robot is supposed to turn around ball",
    //     "GeneralSkills", 0.1, 0.0, 1.0);

    // DoubleParam dribbling_centering_time = createDoubleParam(
    //     "dribbling_centering_time", "the time the ball approximately needs to center inside the dribbler [in s]",
    //     "GeneralSkills", 1.0, 0.0, 10.0);

    // DoubleParam max_vel_drive_dribbling =
    //     createDoubleParam("max_vel_drive_dribbling",
    //                       "sets the robots maximal velocity to dribble with the ball. In relation to the maximal "
    //                       "velocity from the local planner [in %]",
    //                       "GeneralSkills", 0.15, 0.0, 1.0);

    // // defense area distance buffer
    // DoubleParam defense_area_buffer =
    //     createDoubleParam("defense_area_buffer", "Buffer distance for robot from defense area [robot radius +
    //     0.01m]",
    //                       "GeneralSkills", 0.1, 0.09, 0.5);

    // // params for seperate skills

    // // get Ball
    // DoubleParam robot_radius =
    //     createDoubleParam("get_ball_robot_radius", "Radius to retrieve the ball [in m]", "getBall", 0.025, 0.0, 1.0);

    // // Backwards dribbling
    // DoubleParam dist_after_dribbling = createDoubleParam(
    //     "dist_after_dribbling", "the distance the robot will move away from the ball after placing it",
    //     "BallPlacementDribbling", 0.4, 0.0, 2.0);

    // DoubleParam wait_after_dribbling =
    //     createDoubleParam("wait_after_dribbling", "the time the robot will wait after placing it",
    //                       "BallPlacementDribbling", 2.0, 0.0, 10.0);

    // DoubleParam dribbling_wait_after_dribbler_high =
    //     createDoubleParam("wait_after_dribbler_high", "wait after ball in dribbler and speed set to high",
    //                       "BallPlacementDribbling", 0.1, 0.0, 10.0);

    // // MarkEnemyToBall
    // DoubleParam mark_radius_enemy = createDoubleParam(
    //     "mark_radius_enemy", "Constant radius distance to enemy while marking it [in m]", "MarkEnemy", 0.5,
    //     0.18, 3.0);

    // // BlockEnemyLineOfSight
    // DoubleParam block_enemy_line_of_sight_radius = createDoubleParam(
    //     "block_enemy_line_of_sight_radius",
    //     "Constant radius distance to ball while blocking line of sight from the ball carrier enemy [in m]",
    //     "BlockEnemyLineOfSight", 0.3, 0.1, 1);

    // DoubleParam circle_weight = createDoubleParam(
    //     "circle_weight",
    //     "Weight from the circle must be lower than from the line when we want to get faster on the line",
    //     "CircleWeight", 0.9, 0.1, 1);

    // // Run free
    // DoubleParam rf_influence_distance_circle =
    //     createDoubleParam("influence_distance_circle", "", "RunFree", 0.3, 0.0, 20.0);

    // DoubleParam rf_weight_center = createDoubleParam("rf_weight_center", "", "RunFree", 1.0, 0.0, 1.0);

    // DoubleParam rf_influence_distance_enemy_robot =
    //     createDoubleParam("influence_distance_enemy_robot", "", "RunFree", 0.5, 0.0, 20.0);

    // DoubleParam rf_enemy_radius =
    //     createDoubleParam("radius_anittargetfeature_enemy_robot", "", "RunFree", 0.1, 0.0, 1.0);

    // // KickBallToPoint
    // DoubleParam dynamic_friction = createDoubleParam("dynamic_friction", "dynamic friction from the ball while
    // kicking",
    //                                                  "KickBallToPoint", 2.5, 0.0, 6.5);

    // DoubleParam static_friction = createDoubleParam("static_friciton", "static friction from the ball while kicking",
    //                                                 "KickBallToPoint", 2.5, 0.0, 6.5);

    // // kick ball through target
    // DoubleParam kick_ball_through_target_preposition_angle =
    //     createDoubleParam("kick_ball_through_target_preposition_angle", "angle of arc shape for preposition",
    //                       "KickBallThroughTarget", L_PI / 2, 0.0, 2 * L_PI);

    // // ball palcement
    // DoubleParam ball_placement_avoid_speed = createDoubleParam(
    //     "ball_placement_avoid_speed", "angle of arc shape for preposition", "BallPlacementAvoid", 1.0, 0.0, 3.0);

    // // pass ball to robot
    // DoubleParam kick_vel_offset =
    //     createDoubleParam("kick_vel_offset", "velocity offset for passes", "PassBallToRobot", 2.735, 0.0, 6.5);
    // DoubleParam kick_vel_multiplier = createDoubleParam("kick_vel_multiplier", "velocity multiplier for passes",
    //                                                     "PassBallToRobot", 0.22269, 0.0, 1.0);
    // // ReceiveBall
    // DoubleParam receive_ball_kg = createDoubleParam("receive_ball_kg", "kg for drivestep", "ReceiveBall", 0.02, 0.0,
    // 2); DoubleParam receive_ball_kv = createDoubleParam("receive_ball_kv", "kv for drivestep", "ReceiveBall", 0.01,
    // 0.0, 2); DoubleParam receive_ball_margin = createDoubleParam(
    //     "receive_ball_margin",
    //     "margin from the field lines where the robot should no longer move backwards to absorb the ball",
    //     "ReceiveBall", 0.2, 0.0, 1.5);

    // // PrepareKick
    // DoubleParam prepare_kick_max_vel_x =
    //     createDoubleParam("max_vel_x", "Maximum velocity in x direction", "PrepareKick", 1.0);

    // DoubleParam prepare_kick_max_vel_y =
    //     createDoubleParam("max_vel_y", "Maximum velocity in y direction", "PrepareKick", 1.0);

    // // ReflexKick
    // DoubleParam reflex_kick_dribbler_offset =
    //     createDoubleParam("reflex_kick_dribbler_offset", "Offset from dribbler", "ReflexKick", 0.09);
    // // DefendGoalOnCircle

    // DoubleParam defend_goal_on_circle_enemy_threshold =
    //     createDoubleParam("defend_goal_circle_enemy_threshold",
    //                       "Distance an enemy needs to have to the ball to be recognized as ball carrier",
    //                       "DefendGoalOnCircle", 1.0, 0.0, 5.0);

    // DoubleParam defend_goal_on_circle_line_weight =
    //     createDoubleParam("defend_goal_circle_line_weight", "Weight of the line", "DefendGoalOnCircle", 1.0,
    //     0.0, 1.1);

    // DoubleParam defend_goal_on_circle_threshold =
    //     createDoubleParam("goal_threshold", "Threshold", "DefendGoalOnCircle", 1.0, 0.0, 3.0);

    // DoubleParam defend_goal_on_circle_closest_robot_dist =
    //     createDoubleParam("closest_robot_dist", "Closest robot distance", "DefendGoalOnCircle", 0.5, 0.0, 1.0);

    // DoubleParam defend_goal_on_circle_vel_threshold =
    //     createDoubleParam("vel_threshold", "Velocity Threshold", "DefendGoalOnCircle", 1.0, 0.0, 3.0);

    // DoubleParam half_goal_width =
    //     createDoubleParam("half_goal_width", "Half goal width for cutoff during lineofsight defense",
    //                       "DefendGoalOnCircle", 0.35, 0.0, 1.0);

    // // MoveToPenaltyLine
    // DoubleParam influence_dist_antigoal = createDoubleParam(
    //     "influence_dist_antigoal", "Tunes the influence distance of the antigoals defined as other robots",
    //     "MoveToPenaltyLine", 1.0, 0.0, 5.0);
    // DoubleParam antigial_rad = createDoubleParam("antigoal_rad", "radius of antigoals defined as other robots",
    //                                              "MoveToPenaltyLine", 0.09, 0.0, 1.0);

    // // GoalieIntercept
    // DoubleParam goalie_intercept_goal_distance =
    //     createDoubleParam("goalie_intercept_goal_distance", "Max offset to goal the ball line can have",
    //                       "GoalieIntercept", 1.0, 0.0, 3.0);

    // // Goalie LOS defend without ball
};

}  // namespace luhsoccer::config_provider