#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

struct GameConfig : public Config {
    GameConfig() : Config("game.toml", datatypes::ConfigType::SHARED) {}

    BoolParam is_blue = createBoolParam("is_blue", "Do we play as blue our yellow?", "general", true);
    BoolParam is_flipped = createBoolParam("is_flipped", "Should the game be flipped?", "general", false);

    BoolParam robot_controllers_active =
        createBoolParam("robot_controllers_active", "Should the robot controllers be active", "Controller", true);
    BoolParam strategy_active = createBoolParam("strategy_active", "Should the strategy be active", "Controller", true);
    StringParam active_robot_controllers =
        createStringParam("active_robot_controllers", "List of robot ids, separated by commas", "Controller",
                          "0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15");
};

}  // namespace luhsoccer::config_provider