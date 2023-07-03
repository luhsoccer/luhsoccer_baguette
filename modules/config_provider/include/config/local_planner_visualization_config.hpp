#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

// Create Struct which extends the _Config struct_
// Give the constructor of the Config struct the name of the .toml file (also include the .toml extension)
struct LocalPlannerVisualizationConfig : public Config {
    LocalPlannerVisualizationConfig() : Config("local_planner_visualization.toml") {}

    // Create a Parameter (here it is a Boolean Parameter) using the _createBoolParam_ method
    // Arguments are:
    //      1. The _toml_key_: This key has to be unique for every parameter in this Config
    //      2. The _description_: A description for this parameter
    //      3. The group this parameter will be stored in (put an empty string for the global group)
    //      4. The default value of this parameter (used when no value was found in the config)
    //      (5 & 6. For Int & Double Parameters (optional): The min & max value)
    //      7. (optional): Whether the variable should be editable at runtime (defaults to true)
    // shapes
    BoolParam shapes_display = createBoolParam("shapes_display", "Display the active shapes.", "Shapes", true);
    BoolParam shapes_display_influence = createBoolParam(
        "shapes_display_influence", "Make shapes translucent if they have no influence.", "Shapes", true);
    DoubleParam shapes_influence_value = createDoubleParam(
        "shapes_influence_value", "Transparency if shape has no influence (0 = invisible, 1 = visible).", "Shapes", 0.5,
        0.0, 1.0);
    BoolParam shapes_vector_display =
        createBoolParam("shapes_vector_display", "Display the vectors to active shapes.", "Shapes", false);
    BoolParam shapes_vector_to_center_display = createBoolParam(
        "shapes_vector_to_center_display", "Display the vectors to the centers to active shapes.", "Shapes", false);
    // path planning
    BoolParam rotation_vectors_display =
        createBoolParam("rotation_vectors_display", "Display the current rotation vectors.", "Path planning", false);
    BoolParam best_simulation_result_display = createBoolParam(
        "best_simulation_result_display", "Display the best simulation result.", "Path planning", false);
    BoolParam simulation_result_display =
        createBoolParam("simulation_result_display", "Display all simulation results.", "Path planning", false);
    BoolParam failed_simulation_result_display = createBoolParam(
        "failed_simulation_result_display", "Display also failed simulation results.", "Path planning", false);
    BoolParam simulation_result_score_display = createBoolParam(
        "simulation_result_score_display", "Display the score of simulation results.", "Path planning", true);
    DoubleParam simulation_result_lifetime = createDoubleParam(
        "simulation_result_lifetime", "Duration in s for displaying simulation results (0 = infinite).",
        "Path planning", 2.0, 0.0, 100.0);

    BoolParam plot_translation_vlc = createBoolParam(
        "plot_translation_vlc", "Plot desired and current velocity and force of vlc.", "Plotting", false);
    BoolParam display_rotation_vlc = createBoolParam(
        "display_rotation_vlc", "Plot desired and current velocity and force of vlc.", "Plotting", false);
};

}  // namespace luhsoccer::config_provider