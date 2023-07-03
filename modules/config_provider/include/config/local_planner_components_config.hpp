#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

//  Create Struct which extends the _Config struct_
//  Give the constructor of the Config struct the name of the .toml file (also include the .toml extension)
struct LocalPlannerComponentsConfig : public Config {
    LocalPlannerComponentsConfig() : Config("local_planner_components.toml", datatypes::ConfigType::SHARED) {}

    // Create a Parameter (here it is a Boolean Parameter) using the _createBoolParam_ method
    // Arguments are:
    //      1. The _toml_key_: This key has to be unique for every parameter in this Config
    //      2. The _description_: A description for this parameter
    //      3. The group this parameter will be stored in (put an empty string for the global group)
    //      4. The default value of this parameter (used when no value was found in the config)
    //      (5 & 6. For Int & Double Parameters (optional): The min & max value)
    //      7. (optional): Whether the variable should be editable at runtime (defaults to true)
    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers)
    // local planner
    IntParam controller_freq = createIntParam("controller_freq", "Frequency of the controller", "local_planner", 50);
    // robot
    // max velocity
    DoubleParam robot_vel_max_x = createDoubleParam(
        "robot_vel_max_x", "Maximum absolute velocity of the robot in forward direction", "robot", 3.0);
    DoubleParam robot_vel_max_y = createDoubleParam(
        "robot_vel_max_y", "Maximum absolute velocity of the robot in sideways direction", "robot", 3.0);
    DoubleParam robot_vel_max_theta =
        createDoubleParam("robot_vel_max_theta", "Maximum absolute rotational velocity", "robot", 4.0);

    DoubleParam robot_radius =
        createDoubleParam("robot_radius", "Radius of the robot in m", "robot", 0.09, 0.0, 1.0, false);

    DoubleParam max_vel_at_collision = createDoubleParam(
        "max_vel_at_collision", "Maximum absolute velocity of the robot in proxitimty to obstacles.", "robot", 1.0);

    // max acceleration
    DoubleParam robot_acc_max_x =
        createDoubleParam("robot_acc_max_x", "Maximum acceleration of the robot in forward direction", "robot", 5.0);
    DoubleParam robot_acc_max_y =
        createDoubleParam("robot_acc_max_y", "Maximum acceleration of the robot in sideways direction", "robot", 5.0);
    DoubleParam robot_acc_max_theta =
        createDoubleParam("robot_acc_max_theta", "Maximum rotational acceleration", "robot", 60.0);
    // max break
    DoubleParam robot_brk_max_x =
        createDoubleParam("robot_brk_max_x", "Maximum deceleration of the robot in forward direction", "robot", 5.0);
    DoubleParam robot_brk_max_y =
        createDoubleParam("robot_brk_max_y", "Maximum deceleration of the robot in sideways direction", "robot", 5.0);
    DoubleParam robot_brk_max_theta =
        createDoubleParam("robot_brk_max_theta", "Maximum rotational deceleration", "robot", 60.0);

    // rotation control
    DoubleParam rotation_k_g =
        createDoubleParam("rotation_k_g", "Position gain of the VLC for rotation", "rotation", 4);
    DoubleParam rotation_k_v =
        createDoubleParam("rotation_k_v", "Velocity gain of the VLC for rotation", "rotation", 0.7);

    DoubleParam ball_y_vel_from_rotation = createDoubleParam(
        "ball_y_vel_from_rotation", "Factor for Velocity in y direction to rotation velocity to turn around ball.",
        "rotation", 0.1, -1.0, 1.0);
    DoubleParam ball_x_vel_from_rotation = createDoubleParam(
        "ball_x_vel_from_rotation", "Factor for Velocity in y direction to rotation velocity to turn around ball.",
        "rotation", 0.0, -1.0, 1.0);
    DoubleParam robot_vel_max_theta_with_ball =
        createDoubleParam("robot_vel_max_theta_with_ball", "Maximum absolute rotational velocity", "rotation", 3.0);

    DoubleParam robot_vel_max_theta_while_moving =
        createDoubleParam("robot_vel_max_theta_while_moving",
                          "Maximum absolute rotational velocity while tobot is moving.", "rotation", 1.0);
    DoubleParam robot_moving_threshold =
        createDoubleParam("robot_moving_threshold", "Min velocity to be moving.", "rotation", 2.0);
    // features
    DoubleParam feature_target_k_g =
        createDoubleParam("feature_target_k_g", "Position gain of the VLC", "feature_target", 210);

    DoubleParam feature_target_k_v =
        createDoubleParam("feature_target_k_v", "Velocity gain of the VLC", "feature_target", 110);

    DoubleParam feature_robot_obstacle_influence_distance = createDoubleParam(
        "feature_robot_obstacle_influence_distance", "Maximum distance for a robot cd obstacle to influence the robot",
        "feature_cf_obstacle", 1.5);

    DoubleParam feature_robot_obstacle_ataka_influence_distance = createDoubleParam(
        "feature_robot_obstacle_ataka_influence_distance",
        "Maximum distance for a robot cf obstacle to apply the ataka force to the robot", "feature_cf_obstacle", 0.5);

    DoubleParam feature_robot_obstacle_k_cf =
        createDoubleParam("feature_robot_obstacle_k_cf", "Gain in robot cf obstacles of the circular fields force",
                          "feature_cf_obstacle", 110);

    DoubleParam feature_robot_obstacle_k_ataka =
        createDoubleParam("feature_robot_obstacle_k_ataka", "Gain in robot cf obstacles of the ataka repelling force",
                          "feature_cf_obstacle", 110);

    DoubleParam feature_robot_obstacle_k_apf =
        createDoubleParam("feature_robot_obstacle_k_apf", "Gain in robot apf obstacles", "feature_cf_obstacle", 1.0);

    DoubleParam feature_robot_obstacle_k_gyro =
        createDoubleParam("feature_robot_obstacle_k_gyro", "Gain in robot gyro obstacles", "feature_cf_obstacle", 1.0);

    DoubleParam feature_anti_target_k = createDoubleParam(
        "feature_anti_target_k", "Global gain for repell force from anti targets", "feature_anti_target", 0.5);

    DoubleParam feature_turn_around_ball_k =
        createDoubleParam("feature_turn_around_ball_k", "Gain for velocity of robot", "feature_target", 0.2);
    DoubleParam feature_turn_around_ball_g = createDoubleParam(
        "feature_turn_around_ball_g", "Gain for correction of desired velocity for keeping the same distance to ball",
        "feature_target", 0.25);
    DoubleParam feature_turn_around_ball_max_vel_x =
        createDoubleParam("feature_turn_around_ball_max_vel_x", "Maximum velocity in x direction for turn around ball",
                          "feature_target", 1.0);
    DoubleParam feature_turn_around_ball_max_vel_y =
        createDoubleParam("feature_turn_around_ball_max_vel_y", "Maximum velocity in y direction for turn around ball",
                          "feature_target", 1.0);
    DoubleParam obstacle_collision_distance = createDoubleParam(
        "obstacle_collision_distance", "distance of a collision to an obstacle", "feature_obstacles", 0.5, 0, 1.0);
    DoubleParam defense_area_collision_distance =
        createDoubleParam("defense_area_collision_distance", "Distance to defense area to considered as collision",
                          "feature_obstacles", 0.5, 0, 3.0);
    // step
    DoubleParam step_drive_predict_duration_ms = createDoubleParam(
        "step_drive_predict_duration_ms",
        "Duration in ms that the drive step predicts in order to obtain to a target velocity and position",
        "drive_step", 45);

    // simulation
    DoubleParam simulation_max_deviation = createDoubleParam(
        "simulation_max_deviation", "Maximum distance from the current real position to the simulation to be vaild.",
        "simulation", 0.3);

    DoubleParam simulation_acc_k_p = createDoubleParam(
        "simulation_acc_k_p", "P Gain for acceleration control of the internal simulator.", "simulation", 0.5);

    DoubleParam simulation_acc_k_v = createDoubleParam(
        "simulation_acc_k_v", "Constant acceleration gain to compensate for friction.", "simulation", 1.2);

    DoubleParam simulation_acc_scale =
        createDoubleParam("simulation_acc_scale", "Multiplier for the maximum acceleration.", "simulation", 2.0);

    DoubleParam simulation_mass = createDoubleParam("simulation_mass", "Mass of the robot.", "simulation", 1.5);
    DoubleParam simulation_friction =
        createDoubleParam("simulation_friction", "Friction of the robot.", "simulation", 0.22);

    DoubleParam step_drive_w1_alpha =
        createDoubleParam("step_drive_w1_alpha", "Alpha factor from relaxation factor w1.", "drive_step", 0.3);
    DoubleParam setp_drive_min_relaxation_factor =
        createDoubleParam("setp_drive_min_relaxation_factor", "Minimum of the relaxation factor.", "drive_step", 0.2);

    DoubleParam stop_state_min_ball_distance = createDoubleParam(
        "stop_state_min_ball_distance", "Minimum distance to the ball in stop state.", "stop_state", 0.5);
    DoubleParam stop_state_min_line_distance = createDoubleParam(
        "stop_state_min_line_distance", "Minimum distance to field lines in stop state.", "stop_state", 0.15);

    DoubleParam stop_state_max_speed =
        createDoubleParam("stop_state_max_speed", "Maximum speed of robot in stop state", "stop_state", 1.5);
    // kick step
    DoubleParam step_kick_voltage_k =
        createDoubleParam("step_kick_voltage_k", "Amount of voltage for the kick", "kick_step", 0.0358);
    DoubleParam step_kick_voltage_min =
        createDoubleParam("step_kick_voltage_min", "Minimum voltage for the kick", "kick_step", 50);
    DoubleParam step_kick_voltage_max =
        createDoubleParam("step_kick_voltage_max", "Maximum voltage for the kick", "kick_step", 187);
    DoubleParam step_kick_velocity_offset =
        createDoubleParam("step_kick_velocity_offset", "Offset of the kick's velocity", "kick_step", -0.7729);

    DoubleParam side_decision_neighbour_distance =
        createDoubleParam("side_decision_neighbour_distance",
                          "Maximum distance of two robot to be considered as neighbors.", "drive_step", 0.5);

    DoubleParam simulation_scoring_collision_distance =
        createDoubleParam("simulation_scoring_collision_distance",
                          "Maximum distance between robots to be considered as collision.", "simulation_scoring", 0.2);

    DoubleParam simulation_scoring_duration_k =
        createDoubleParam("simulation_scoring_duration_k", "Multiplier for the weight of the duration part in scoring.",
                          "simulation_scoring", 1000.0);
    DoubleParam simulation_scoring_obstacle_distance_k = createDoubleParam(
        "simulation_scoring_obstacle_distance_k", "Multiplier for the weight of the obstacle distance part in scoring.",
        "simulation_scoring", 10000.0);

    BoolParam stop_state = createBoolParam("stop_state", "stop state", "state", false);

    // avoidance modes

    StringParam avoid_force_mode =
        createStringParam("avoid_force_mode", "Avoid force mode. Choose from: CIRCULAR_FIELDS, APF, ATAKA, GYROSCOPIC",
                          "AvoidanceMode", "CIRCULAR_FIELDS");

    StringParam magentic_field_vector_mode =
        createStringParam("magentic_field_vector_mode",
                          "Choose from: OFF, MULTI_AGENT, SIDE_DECISION, COOPERATIVE_SIDE_DECISION, HADDADIN",
                          "AvoidanceMode", "COOPERATIVE_SIDE_DECISION");

    IntParam avoid_force_mode_num = createIntParam(
        "avoid_force_mode_num", "Avoid force mode. Choose from: 0=CIRCULAR_FIELDS, 1=APF, 2=ATAKA, 3=GYROSCOPIC",
        "AvoidanceMode", 0, 0, 3);

    IntParam magentic_field_vector_mode_num =
        createIntParam("magentic_field_vector_mode_num",
                       "Choose from: 0=OFF, 1=MULTI_AGENT, 2=SIDE_DECISION, 3=COOPERATIVE_SIDE_DECISION, 4=HADDADIN",
                       "AvoidanceMode", 3, 0, 4);

    DoubleParam defense_area_margin =
        createDoubleParam("defense_area_margin", "Margin of defense area.", "defense_area", 0.0, 0.0, 1.0);

    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
};

}  // namespace luhsoccer::config_provider