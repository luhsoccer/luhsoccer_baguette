#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

// Create Struct which extends the _Config struct_
// Give the constructor of the Config struct the name of the .toml file (also include the .toml extension)
struct BallFilterConfig : public Config {
    BallFilterConfig() : Config("ball_filter.toml") {}

    // Create a Parameter (here it is a Boolean Parameter) using the _createBoolParam_ method
    // Arguments are:
    //      1. The _toml_key_: This key has to be unique for every parameter in this Config
    //      2. The _description_: A description for this parameter
    //      3. The group this parameter will be stored in (put an empty string for the global group)
    //      4. The default value of this parameter (used when no value was found in the config)
    //      (5 & 6. For Int & Double Parameters (optional): The min & max value)
    //      7. (optional): Whether the variable should be editable at runtime (defaults to true)
    DoubleParam speed_filter_max_speed = createDoubleParam(
        "max_ball_speed", "The max. valid speed for a ball using the ball speed filter. (m/s)", "speed_filter", 7.0);
    BoolParam use_moving_average_filter = createBoolParam(
        "use_moving_average_filter", "Whether to use the moving average filter", "moving_average_filter", false);
    BoolParam use_speed_filter =
        createBoolParam("use_speed_filter", "Whether to use the speed filter", "speed_filter", true);
    IntParam moving_average_filter_size = createIntParam(
        "moving_average_filter_size", "The size of the moving average filter", "moving_average_filter", 20, 1, 1000);
    DoubleParam moving_average_filter_forgetting_factor =
        createDoubleParam("forgetting_factor", "The forgetting factor of the moving average filter",
                          "moving_average_filter", 0.5, 0.0, 1.0);
    DoubleParam camera_wait_time =
        createDoubleParam("camera_wait_time", "The time to wait for a camera to be available", "filter", 1);
    DoubleParam enemy_robot_dribbler_vision_delay =
        createDoubleParam("enemy_robot_dribbler_vision_delay",
                          "The time we allow the ball not to be seen in the vision", "enemy_vision_filter", 1);
    DoubleParam enemy_dribbler_angle = createDoubleParam(
        "enemy_dribbler_angle",
        "The angle in degrees between the direction of the enemy and the ball, which is considered as inside "
        "the dribbler of the enemy",
        "enemy_vision_filter", 30, 0.0, 360.0);
    DoubleParam enemy_dribbler_distance =
        createDoubleParam("enemy_dribbler_distance",
                          "The distance between the middle of the enemy robot and the ball, which is "
                          "considered as inside the dribbler of the enemy",
                          "enemy_vision_filter", 0.12, 0.0, 10.0);
    DoubleParam ally_dribbler_angle = createDoubleParam(
        "ally_dribbler_angle",
        "The angle in degrees between the direction of the ally and the ball, which is considered as inside "
        "the dribbler of the ally",
        "ally_vision_filter", 30, 0.0, 360.0);
    DoubleParam ally_dribbler_distance =
        createDoubleParam("ally_dribbler_distance",
                          "The distance between the middle of the ally robot and the ball, which is "
                          "considered as inside the dribbler of the ally",
                          "ally_vision_filter", 0.075, 0.0, 10.0);
    DoubleParam ball_velocity_threshold = createDoubleParam(
        "ball_velocity_threshold", "The velocity threshold for the ball to be considered as moving", "filter", 0.15);
    DoubleParam ally_robot_dribbler_feedback_threshold = createDoubleParam(
        "ally_robot_dribbler_feedback_threshold",
        "Threshold after which the ally dribbler feedback starts to rely on vision after robot feedback cutoff",
        "ally_vision_filter", 0.15);

    DoubleParam light_barrier_low_pass_time = createDoubleParam(
        "light_barrier_low_pass_time",
        "Duration in s for which the ball is still considered in dribbler, after light barrier is false.",
        "ally_vision_filter", 0.1);

    BoolParam ignore_ally_dribbler_feedback =
        createBoolParam("ignore_ally_dribbler_feedback",
                        "Whether to ignore the dribbler feedback from the ally robot and rely only on vision",
                        "ally_vision_filter", false);
};
}  // namespace luhsoccer::config_provider
