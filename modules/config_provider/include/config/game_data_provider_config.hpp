#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

//  Create Struct which extends the _Config struct_
//  Give the constructor of the Config struct the name of the .toml file (also include the .toml extension)
struct GameDataProviderConfig : public Config {
    GameDataProviderConfig() : Config("game_data_provider_shared.toml", datatypes::ConfigType::SHARED) {}

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
    // robot

    // max velocity
    DoubleParam kalman_r_stddev_x = createDoubleParam(
        "kalman_r_stddev_x", "standard deviation of x position measurement of the robot", "kalman", 0.0013, 0.0, 1.0);
    DoubleParam kalman_r_stddev_y = createDoubleParam(
        "kalman_r_stddev_y", "standard deviation of y position measurement of the robot", "kalman", 0.0013, 0.0, 1.0);
    DoubleParam kalman_r_stddev_theta = createDoubleParam(
        "kalman_r_stddev_theta", "standard deviation of rotation measurement of the robot", "kalman", 0.01, 0.0, 1.0);
    DoubleParam kalman_r_stddev_vx = createDoubleParam(
        "kalman_r_stddev_vx", "standard deviation of rotation measurement of the robot", "kalman", 1000.0, 0.0, 1000.0);
    DoubleParam kalman_r_stddev_vy =
        createDoubleParam("kalman_r_stddev_vy", "standard deviation of y velocity measurement of the robot ", "kalman",
                          1000.0, 0.0, 1000.0);
    DoubleParam kalman_r_stddev_vtheta =
        createDoubleParam("kalman_r_stddev_vtheta", "standard deviation of rotation velocity measurement of the robot ",
                          "kalman", 1000.0, 0.0, 1000.0);
    DoubleParam kalman_q_stddev_x = createDoubleParam(
        "kalman_q_stddev_x", "standard deviation of x position process of the robot ", "kalman", 0.01, 0.0, 1.0);
    DoubleParam kalman_q_stddev_y = createDoubleParam(
        "kalman_q_stddev_y", "standard deviation of y velocity process of the robot ", "kalman", 0.4, 0.0, 1.0);
    DoubleParam kalman_q_stddev_theta = createDoubleParam(
        "kalman_q_stddev_theta", "standard deviation of rotation process of the robot ", "kalman", 0.03, 0.0, 1.0);
    DoubleParam kalman_q_stddev_vx = createDoubleParam(
        "kalman_q_stddev_vx", "standard deviation of x velocity process of the robot ", "kalman", 0.4, 0.0, 1.0);
    DoubleParam kalman_q_stddev_vy = createDoubleParam(
        "kalman_q_stddev_vy", "standard deviation of y velocity process of the robot", "kalman", 0.4, 0.0, 1.0);
    DoubleParam kalman_q_stddev_vtheta =
        createDoubleParam("kalman_q_stddev_vtheta", "standard deviation of rotation velocity process of the robot ",
                          "kalman", 0.4, 0.0, 1.0);
    BoolParam kalman_correction = createBoolParam(
        "kalman_correction", "If the kalman filter should correct the position with vision data.", "kalman", true);

    DoubleParam kalman_k_p =
        createDoubleParam("kalman_k_p", "gain of velocity error in system model", "kalman", 0.5, 0.0, 1.0);
    DoubleParam kalman_k_v =
        createDoubleParam("kalman_k_v", "gain of velocity in system model", "kalman", 1.2, 0.0, 5.0);
    DoubleParam kalman_acc_scale = createDoubleParam("kalman_acc_scale", "scale for max acceleration", "kalman", 2.0);

    IntParam vision_delay_ms = createIntParam("vision_delay_ms", "Delay of vision commands in ms.", "kalman", 35);
    BoolParam show_true_robot_position = createBoolParam(
        "show_true_robot_position",
        "Publish the true robot position to the worldmodel. Only works in correct feedback mode of simulation!",
        "kalman", false);

    BoolParam show_vision_robot_position = createBoolParam(
        "show_vision_robot_position", "Publish the vision robot position to the worldmodel.", "kalman", false);

    BoolParam use_robot_feedback_in_filter = createBoolParam(
        "use_robot_feedback_in_filter", "Use the feedback of the robots to filter the position.", "kalman", false);

    DoubleParam state_change_distance = createDoubleParam(
        "state_change_ball_travel_distance", "The distance after the state machine auto", "state_machine", 0.05);

    DoubleParam state_change_time = createDoubleParam(
        "state_change_time", "The time after the state machine auto switches to normal play.", "state_machine", 10);

    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
};

}  // namespace luhsoccer::config_provider