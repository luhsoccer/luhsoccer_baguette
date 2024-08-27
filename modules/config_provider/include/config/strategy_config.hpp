#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

struct StrategyConfig : public Config {
    StrategyConfig() : Config("strategy.toml", datatypes::ConfigType::SHARED) {}

    // Goalie
    DoubleParam goalie_defense_area_caution_brim =
        createDoubleParam("defense_area_caution_brim",
                          "Defines an inner brim of the defense area, where the goalie has to be cautious that an "
                          "enemy roobt can steal the ball out of the defense area",
                          "Goalie", 0.2, 0.0, 1.0);

    DoubleParam goalie_defense_area_caution_brim_waittime = createDoubleParam(
        "defense_area_caution_brim_waittime",
        "Defines the time in seconds to wait before getting a ball from the caution brim", "Goalie", 1.0, 0.0, 10.0);

    DoubleParam goalie_ball_in_shot_threshold_velocity = createDoubleParam(
        "ball_in_shot_threshold_velocity", "Defines a minimum velocity for the ball to be considered in a shot or pass",
        "Goalie", 0.3, 0.0, 2.0);

    DoubleParam goalie_kick_ball_out_safety_margin =
        createDoubleParam("kick_ball_out_safety_margin",
                          "Defines an offset in negative x direction when shooting the ball out of the field, so that "
                          "it will not be an aimless kick",
                          "Goalie", 0.2, 0.0, 1.0);
    DoubleParam goalie_shot_on_goal_safety_margin =
        createDoubleParam("shot_on_goal_safety_margin",
                          "Makes the goal bigger so more shots are considered on the goal", "Goalie", 0.2, 0.0, 1.0);
    DoubleParam goalie_los_defend_max_distance =
        createDoubleParam("los_defend_max_distance",
                          "Maximum distance between ball and enemy robot to consider that the enemy is carry the ball",
                          "Goalie", 0.5, 0.0, 1.0);

    DoubleParam goalie_los_defend_aim_threshold = createDoubleParam(
        "los_defend_aim_threshold", "Imaginary half goal size for aims on our goal", "Goalie", 2.0, 0.0, 6.0);

    DoubleParam goalie_max_dist_for_pass_prediction = createDoubleParam(
        "max_dist_for_pass_prediction", "Maximum distance from the pass line to be considered as receive candidate",
        "Goalie", 0.5, 0.0, 2.0);

    IntParam goalie_max_angle_for_dangerous_pass =
        createIntParam("max_angle_for_dangerous_pass",
                       "The maximum angle from the goal to be defined as valid pass candidate", "Goalie", 60, 0, 90);
    BoolParam goalie_predict_passes =
        createBoolParam("predict_passes", "Predict passes and receiver in front of the goal", "Goalie", true);
};

}  // namespace luhsoccer::config_provider