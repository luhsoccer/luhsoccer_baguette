#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

// Create Struct which extends the _Config struct_
// Give the constructor of the Config struct the name of the .toml file (also include the .toml extension)
struct RobotControlConfig : public Config {
    RobotControlConfig() : Config("robot_control.toml", datatypes::ConfigType::SHARED) {}

    // Create a Parameter (here it is a Boolean Parameter) using the _createBoolParam_ method
    // Arguments are:
    //      1. The _toml_key_: This key has to be unique for every parameter in this Config
    //      2. The _description_: A description for this parameter
    //      3. The group this parameter will be stored in (put an empty string for the global group)
    //      4. The default value of this parameter (used when no value was found in the config)
    //      (5 & 6. For Int & Double Parameters (optional): The min & max value)
    //      7. (optional): Whether the variable should be editable at runtime (defaults to true)
    /// @todo description
    // controllers
    IntParam controllers_frequency = createIntParam("controllers_frequency", "", "00 - Controllers", 100);

    // robot limits
    DoubleParam robot_radius = createDoubleParam("robot_radius", "", "01 - Robot", 0.09, 0.0, 0.5);
    DoubleParam robot_max_vel_x = createDoubleParam("robot_max_vel_x", "", "01 - Robot", 2.5, 0.0, 5.0);
    DoubleParam robot_max_vel_y = createDoubleParam("robot_max_vel_y", "", "01 - Robot", 2.5, 0.0, 5.0);
    DoubleParam robot_max_vel_theta = createDoubleParam("robot_max_vel_theta", "", "01 - Robot", 4.0, 0.0, 12.0);
    DoubleParam robot_max_acc_x = createDoubleParam("robot_max_acc_x", "", "01 - Robot", 3.0, 0.0, 10.0);
    DoubleParam robot_max_acc_y = createDoubleParam("robot_max_acc_y", "", "01 - Robot", 1.75, 0.0, 10.0);
    DoubleParam robot_max_acc_theta = createDoubleParam("robot_max_acc_theta", "", "01 - Robot", 100.0, 0.0, 150.0);
    DoubleParam robot_max_brk_x = createDoubleParam("robot_max_brk_x", "", "01 - Robot", 5.46, 0.0, 10.0);
    DoubleParam robot_max_brk_y = createDoubleParam("robot_max_brk_y", "", "01 - Robot", 3.33, 0.0, 10.0);
    DoubleParam robot_max_brk_theta = createDoubleParam("robot_max_brk_theta", "", "01 - Robot", 100.0, 0.0, 150.0);

    // Targets
    DoubleParam target_k_p = createDoubleParam("target_k_p", "", "02 - Gains", 0.1);
    DoubleParam target_min_velocity = createDoubleParam("target_min_velocity", "", "02 - Gains", 0.5);
    // Anti Targets
    DoubleParam anti_target_k_ip = createDoubleParam("anti_target_k_ip", "", "02 - Gains", 0.1);
    // Rotation
    DoubleParam rotation_k_p = createDoubleParam("rotation_k_p", "", "02 - Gains", 0.1);

    // Turn Around Ball
    DoubleParam turn_around_ball_trans_k_p =
        createDoubleParam("turn_around_ball_trans_k_p", "", "03 - Drive with Ball", 0.1);
    DoubleParam turn_around_ball_rot_k_p =
        createDoubleParam("turn_around_ball_rot_k_p", "", "03 - Drive with Ball", 0.1);
    DoubleParam turn_around_ball_max_vel_x =
        createDoubleParam("turn_around_ball_max_vel_x", "", "03 - Drive with Ball", 2.0, 0.0, 5.0);
    DoubleParam turn_around_ball_max_vel_y =
        createDoubleParam("turn_around_ball_max_vel_y", "", "03 - Drive with Ball", 2.0, 0.0, 5.0);
    DoubleParam turn_around_ball_max_vel_theta =
        createDoubleParam("turn_around_ball_max_vel_theta", "", "03 - Drive with Ball", 1.0, 0.0, 5.0);
    DoubleParam turn_with_ball_x_vel_from_rotation =
        createDoubleParam("turn_with_ball_x_vel_from_rotation", "", "03 - Drive with Ball", 0.025);
    DoubleParam turn_with_ball_y_vel_from_rotation =
        createDoubleParam("turn_with_ball_y_vel_from_rotation", "", "03 - Drive with Ball", 0.037);

    // obstacles
    DoubleParam obstacle_k_cf = createDoubleParam("obstacle_k_cf", "", "04 - Obstacle", 0.1);
    DoubleParam obstacle_collision_distance =
        createDoubleParam("obstacle_collision_distance_", "", "04 - Obstacle", 0.1, 0.0, 1.0);
    DoubleParam obstacle_collision_max_vel =
        createDoubleParam("obstacle_collision_max_vel", "", "04 - Obstacle", 1.0, 0.0, 5.0);
    DoubleParam obstacle_max_vel_k = createDoubleParam("obstacle_max_vel_k", "", "04 - Obstacle", 1.0, 0.0, 1.0);
    DoubleParam obstacle_influence_distance =
        createDoubleParam("obstacle_influence_distance", "", "04 - Obstacle", 1.5, 0.0, 10.0);
    DoubleParam obstacle_grouping_distance =
        createDoubleParam("obstacle_grouping_distance", "", "04 - Obstacle", 0.5, 0.0, 10.0);

    BoolParam obstacle_better_grouping = createBoolParam(
        "obstacle_better_grouping", "Use area and shape of obstacles for grouping.", "04 - Obstacle", true);

    BoolParam avoid_defense_area = createBoolParam(
        "avoid_defense_area", "Avoid defense area with all robots except the goalie.", "04 - Obstacle", true);

    DoubleParam obstacle_robot_radius =
        createDoubleParam("obstacle_robot_radius", "Radius of the other robots", "04 - Obstacle", 0.1, 0.0, 0.2);

    // relaxation factors
    DoubleParam rel_factors_min = createDoubleParam("rel_factor_min", "Minimum of the relaxation factor.",
                                                    "05 - Relaxation Factor", 0.4, 0.0, 1.0);
    DoubleParam rel_factors_w1_k =
        createDoubleParam("rel_factors_w1_k", "obstacle_distance", "05 - Relaxation Factor", 1.0, 0.0, 2.0);
    DoubleParam rel_factors_w1_alpha = createDoubleParam(
        "rel_factors_w1_alpha", "Alpha factor from relaxation factor w1.", "05 - Relaxation Factor", 0.3);
    DoubleParam rel_factors_w2_k =
        createDoubleParam("rel_factors_w2_k", "obstacle_direction", "05 - Relaxation Factor", 1.0, 0.0, 2.0);

    // Kicking
    IntParam kick_max_voltage =
        createIntParam("kick_max_voltage", "Maximum kick voltage.", "06 - Kicking", 200, 0, 250);
    DoubleParam kick_bonus =
        createDoubleParam("kick_bonus", "Bonus velocity when kicking on the goal", "06 - Kicking", 1.0, 0, 6.0);
    IntParam kick_min_voltage = createIntParam("kick_min_voltage", "Minimum kick voltage.", "06 - Kicking", 50, 0, 250);
    DoubleParam kick_voltage_k = createDoubleParam("kick_voltage_k", "", "06 - Kicking", 0.0358);
    DoubleParam kick_velocity_offset =
        createDoubleParam("kick_velocity_offset", "Offset of the kick's velocity", "06 - Kicking", -0.7729);

    // Stop State
    DoubleParam stop_state_max_vel =
        createDoubleParam("robot_max_vel_stop_state", "", "07 - Stop State", 1.5, 0.0, 5.0);
    DoubleParam stop_state_min_ball_dist =
        createDoubleParam("stop_state_min_ball_dist", "", "07 - Stop State", 0.8, 0.0, 2.0);
    DoubleParam stop_state_defense_area_margin =
        createDoubleParam("stop_state_defense_area_margin", "", "07 - Stop State", 0.0, 0.0, 5.0);
    DoubleParam stop_state_defense_area_margin_stop =
        createDoubleParam("stop_state_defense_area_margin_stop", "", "07 - Stop State", 0.2, 0.0, 5.0);
    DoubleParam stop_state_free_kick_margin =
        createDoubleParam("stop_state_free_kick_margin", "", "07 - Stop State", 0.2, 0.0, 5.0);

    // Simulation
    DoubleParam simulation_mass = createDoubleParam("simulation_mass", "", "08 - Simulation", 1.5, 0.0, 5.0);

    DoubleParam simulation_friction = createDoubleParam("simulation_friction", "", "08 - Simulation", 0.22, 0.0, 5.0);

    DoubleParam simulation_acc_scale = createDoubleParam("simulation_acc_scale", "", "08 - Simulation", 2.0);

    DoubleParam simulation_acc_k_p = createDoubleParam("simulation_acc_k_p", "", "08 - Simulation", 0.5, 0.0, 1.0);

    DoubleParam simulation_acc_k_v = createDoubleParam("simulation_acc_k_v", "", "08 - Simulation", 1.2, 0.0, 5.0);

    IntParam simulation_frequency = createIntParam("simulation_frequency", "", "08 - Simulation", 100, 0, 100);

    // Testing
    BoolParam send_commands = createBoolParam("send_commands", "Send commands to robots.", "99 - Testing", true);
};

}  // namespace luhsoccer::config_provider