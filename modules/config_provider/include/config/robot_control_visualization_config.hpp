#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

// Create Struct which extends the _Config struct_
// Give the constructor of the Config struct the name of the .toml file (also include the .toml extension)
struct RobotControlVisualizationConfig : public Config {
    RobotControlVisualizationConfig() : Config("robot_control_visualization.toml") {}

    // Create a Parameter (here it is a Boolean Parameter) using the _createBoolParam_ method
    // Arguments are:
    //      1. The _toml_key_: This key has to be unique for every parameter in this Config
    //      2. The _description_: A description for this parameter
    //      3. The group this parameter will be stored in (put an empty string for the global group)
    //      4. The default value of this parameter (used when no value was found in the config)
    //      (5 & 6. For Int & Double Parameters (optional): The min & max value)
    //      7. (optional): Whether the variable should be editable at runtime (defaults to true)
    // shapes
    BoolParam display_shapes = createBoolParam("display_shapes", "Display the active shapes.", "Shapes", true);
    BoolParam display_shape_influence = createBoolParam(
        "display_shape_influence", "Make shapes translucent if they have no influence.", "Shapes", true);
    DoubleParam shape_influence_value = createDoubleParam(
        "shape_influence_value", "Transparency if shape has no influence (0 = invisible, 1 = visible).", "Shapes", 0.5,
        0.0, 1.0);
    BoolParam display_shape_vectors =
        createBoolParam("display_shape_vectors", "Display the vectors to active shapes.", "Shapes", false);
    BoolParam display_shape_vectors_to_center = createBoolParam(
        "display_shape_vectors_to_center", "Display the vectors to the centers to active shapes.", "Shapes", false);

    BoolParam display_valid_goal_candidates =
        createBoolParam("display_valid_goal_candidates", "Display points used to find a valid goal.", "Shapes", false);

    BoolParam display_obstacle_distance_vector = createBoolParam(
        "display_obstacle_distance_vector", "Display the minimal distance between obstacles", "Shapes", false);

    BoolParam display_obstacle_group_centers =
        createBoolParam("display_obstacle_group_centers", "Display the obstacle group centers", "Shapes", false);
    // path planning
    BoolParam display_bypass_directions =
        createBoolParam("display_bypass_directions", "Display the current rotation vectors.", "Path planning", false);

    BoolParam plot_translational =
        createBoolParam("plot_translational", "Plot desired and current translational velocity.", "Plotting", false);
    BoolParam plot_rotational =
        createBoolParam("plot_rotational", "Plot desired and current rotational velocity.", "Plotting", false);

    // simulation
    BoolParam simulate_tasks =
        createBoolParam("simulate_tasks", "Simulate the execution of tasks.", "Simulation", false);

    BoolParam simulate_tasks_continuously = createBoolParam(
        "simulate_tasks_continuously", "Simulate the execution of tasks repeating.", "Simulation", false);
};

}  // namespace luhsoccer::config_provider