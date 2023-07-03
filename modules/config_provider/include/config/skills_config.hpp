#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

struct SkillsConfig : public Config {
    SkillsConfig() : Config("skills.toml", datatypes::ConfigType::SHARED) {}

    // params for genereal skills
    DoubleParam missing_field_edge =
        createDoubleParam("puffer_to_fieldline",
                          "in the testing area the space outside of fieldlines is too small. This parameter is the "
                          "missing distance AND NEEDS TO BE SET TO ZERO FOR THE TOURNAMENT",
                          "GeneralSkills", 0.23);

    DoubleParam translational_tolerance = createDoubleParam(
        "translational_tolerance", "Translational tolerance of target feature [im m]", "GeneralSkills", 0.03);

    DoubleParam move_to_ball_pre_radius = createDoubleParam(
        "move_to_ball_pre_radius", "Radius to ball before retrieving it [in m]", "GeneralSkills", 0.25, 0.0, 1.0);

    DoubleParam ball_filter_vel_threshold = createDoubleParam(
        "ball_filter_vel_threshold", "Threshold of ball velocity vector length, in order to filter out ball variance",
        "GeneralSkills", 0.15, 0.0, 1.0);
    DoubleParam robot_vel_max_theta_turn_around_ball =
        createDoubleParam("robot_vel_max_theta_turn_around_ball",
                          "Velocity for TurnAroundBall Feature when the ball is in dribbler", "robot", 4.0, 0.1, 8.0);

    DoubleParam ball_obstacle_weight = createDoubleParam(
        "ball_obstacle_weight", "Weight of ball obstacle feature when robot is supposed to turn around ball",
        "GeneralSkills", 0.1, 0.0, 1.0);

    DoubleParam dribbling_centering_time = createDoubleParam(
        "dribbling_centering_time", "the time the ball approximately needs to center inside the dribbler [in s]",
        "GeneralSkills", 1.0, 0.0, 10.0);

    DoubleParam max_vel_drive_dribbling =
        createDoubleParam("max_vel_drive_dribbling",
                          "sets the robots maximal velocity to dribble with the ball. In relation to the maximal "
                          "velocity from the local planner [in %]",
                          "GeneralSkills", 0.15, 0.0, 1.0);

    // defense area distance buffer
    DoubleParam defense_area_buffer =
        createDoubleParam("defense_area_buffer", "Buffer distance for robot from defense area [robot radius + 0.01m]",
                          "GeneralSkills", 0.1, 0.09, 0.5);

    // params for seperate skills

    // get Ball
    DoubleParam robot_radius =
        createDoubleParam("get_ball_robot_radius", "Radius to retrieve the ball [in m]", "getBall", 0.025, 0.0, 1.0);

    // Backwards dribbling
    DoubleParam dist_after_dribbling = createDoubleParam(
        "dist_after_dribbling", "the distance the robot will move away from the ball after placing it",
        "BallPlacementDribbling", 0.4, 0.0, 2.0);

    // MarkEnemyToBall
    DoubleParam mark_radius_enemy = createDoubleParam(
        "mark_radius_enemy", "Constant radius distance to enemy while marking it [in m]", "MarkEnemy", 0.3, 0.18, 3.0);

    // Oktoskill
    DoubleParam okto_skill_dist =
        createDoubleParam("okto_skill_distance", "Distance to the edge of the field, from where the skill is canceld",
                          "Oktoskill", 0.3, 0.1, 2.0);

    // BlockEnemyLineOfSight
    DoubleParam block_enemy_line_of_sight_radius = createDoubleParam(
        "block_enemy_line_of_sight_radius",
        "Constant radius distance to ball while blocking line of sight from the ball carrier enemy [in m]",
        "BlockEnemyLineOfSight", 0.3, 0.1, 1);

    DoubleParam circle_weight = createDoubleParam(
        "circle_weight",
        "Weight from the circle must be lower than from the line when we want to get faster on the line",
        "CircleWeight", 0.9, 0.1, 1);

    // Run free
    DoubleParam rf_influence_distance_circle =
        createDoubleParam("influence_distance_circle", "", "RunFree", 0.3, 0.0, 20.0);

    DoubleParam rf_weight_center = createDoubleParam("rf_weight_center", "", "RunFree", 1.0, 0.0, 1.0);

    DoubleParam rf_influence_distance_enemy_robot =
        createDoubleParam("influence_distance_enemy_robot", "", "RunFree", 0.5, 0.0, 20.0);

    DoubleParam rf_enemy_radius =
        createDoubleParam("radius_anittargetfeature_enemy_robot", "", "RunFree", 0.1, 0.0, 1.0);

    // KickBallToPoint
    DoubleParam dynamic_friction = createDoubleParam("dynamic_friction", "dynamic friction from the ball while kicking",
                                                     "KickBallToPoint", 2.5, 0.0, 6.5);

    DoubleParam static_friction = createDoubleParam("static_friciton", "static friction from the ball while kicking",
                                                    "KickBallToPoint", 2.5, 0.0, 6.5);

    // kick ball through target
    DoubleParam kick_ball_through_target_preposition_angle =
        createDoubleParam("kick_ball_through_target_preposition_angle", "angle of arc shape for preposition",
                          "KickBallThroughTarget", L_PI / 2, 0.0, 2 * L_PI);

    // pass ball to robot
    DoubleParam kick_vel_offset =
        createDoubleParam("kick_vel_offset", "velocity offset for passes", "PassBallToRobot", 2.735, 0.0, 6.5);
    DoubleParam kick_vel_multiplier = createDoubleParam("kick_vel_multiplier", "velocity multiplier for passes",
                                                        "PassBallToRobot", 0.22269, 0.0, 1.0);
    // ReceiveBall
    DoubleParam receive_ball_kg = createDoubleParam("receive_ball_kg", "kg for drivestep", "ReceiveBall", 0.02, 0.0, 2);
    DoubleParam receive_ball_kv = createDoubleParam("receive_ball_kv", "kv for drivestep", "ReceiveBall", 0.01, 0.0, 2);

    // PrepareKick
    DoubleParam prepare_kick_max_vel_x =
        createDoubleParam("max_vel_x", "Maximum velocity in x direction", "PrepareKick", 1.0);

    DoubleParam prepare_kick_max_vel_y =
        createDoubleParam("max_vel_y", "Maximum velocity in y direction", "PrepareKick", 1.0);

    // ReflexKick
    DoubleParam reflex_kick_dribbler_offset =
        createDoubleParam("reflex_kick_dribbler_offset", "Offset from dribbler", "ReflexKick", 0.09);
    // DefendGoalOnCircle
    DoubleParam defend_goal_on_circle_offset =
        createDoubleParam("circle_offset", "Offset of the circle", "DefendGoalOnCircle", 1.0, 0.0, 4.0);

    DoubleParam defend_goal_on_circle_radius =
        createDoubleParam("circle_radius", "Radius of the circle", "DefendGoalOnCircle", 1.44, 0.0, 4.0);

    DoubleParam defend_goal_on_circle_weight =
        createDoubleParam("defend_goal_circle_weight", "Weight of the circle", "DefendGoalOnCircle", 0.4, 0.0, 1.0);

    DoubleParam defend_goal_on_circle_threshold =
        createDoubleParam("goal_threshold", "Threshold", "DefendGoalOnCircle", 1.0, 0.0, 3.0);

    DoubleParam defend_goal_on_circle_closest_robot_dist =
        createDoubleParam("closest_robot_dist", "Closest robot distance", "DefendGoalOnCircle", 0.5, 0.0, 1.0);

    DoubleParam defend_goal_on_circle_vel_threshold =
        createDoubleParam("vel_threshold", "Velocity Threshold", "DefendGoalOnCircle", 1.0, 0.0, 3.0);

    DoubleParam half_goal_width =
        createDoubleParam("half_goal_width", "Half goal width for cutoff during lineofsight defense",
                          "DefendGoalOnCircle", 0.35, 0.0, 1.0);
};

}  // namespace luhsoccer::config_provider