#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

struct GameConfig : public Config {
    GameConfig() : Config("game.toml", datatypes::ConfigType::SHARED) {}

    BoolParam is_blue = createBoolParam("is_blue", "Do we play as blue our yellow?", "general", true);
    BoolParam is_flipped = createBoolParam("is_flipped", "Should the game be flipped?", "general", false);
};

}  // namespace luhsoccer::config_provider