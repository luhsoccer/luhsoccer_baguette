#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {
struct BallFilterConfig : public Config {
    BallFilterConfig() : Config("ball_filter.toml", datatypes::ConfigType::SHARED) {}

    DoubleParam light_barrier_debounce_time = createDoubleParam(
        "light_barrier_debounce_time", "Time in seconds to debounce the light barrier (requires restart)",
        "light_barrier", 0.1, 0.0);

    IntParam velocity_average_window_size =
        createIntParam("velocity_average_window_size", "Size of the window for the velocity average (requires restart)",
                       "velocity", 10, 1, 200);

    DoubleParam velocity_threshold =
        createDoubleParam("velocity_threshold", "Threshold for the velocity to be considered as moving", "velocity",
                          6.5 * 1.2, 0.0, 6.5 * 3.0);

    DoubleParam light_barrier_velocity_threshold =
        createDoubleParam("light_barrier_velocity_threshold",
                          "Max velocity between the possible ball path of to light barriers between two robots",
                          "light_barrier", 6.5 * 1.4, 0.0, 6.5 * 3.0);

    DoubleParam max_timeout_time =
        createDoubleParam("max_timeout_time", "Max blackout time after a robot feedback will be discarded",
                          "light_barrier", 2.0, 0.0, 10.0);

    DoubleParam dribbler_width =
        createDoubleParam("dribbler_width", "Width of the dribbler", "vision_light_barrier_simulation", 0.1, 0.0);

    DoubleParam x_margin =
        createDoubleParam("x_margin", "Margin in x direction", "vision_light_barrier_simulation", 0.1, 0.0);
    DoubleParam behind_robot_margin = createDoubleParam("behind_robot_margin", "Margin behind the robot",
                                                        "vision_light_barrier_simulation", 0.05, 0.0);
    DoubleParam y_debounce_margin = createDoubleParam("y_debounce_margin", "Margin in y direction for debounce",
                                                      "vision_light_barrier_simulation", 0.015, 0.0);
    DoubleParam x_debounce_margin = createDoubleParam("x_debounce_margin", "Margin in x direction for debounce",
                                                      "vision_light_barrier_simulation", 0.05, 0.0);
};
}  // namespace luhsoccer::config_provider
