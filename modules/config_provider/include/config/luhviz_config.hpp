#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

struct LuhvizConfig : public Config {
    LuhvizConfig() : Config("luhviz.toml", datatypes::ConfigType::LOCAL) {}

    const std::string luhviz_group = "luhviz";

    static constexpr int MIN_ANTIALIASING = 0;
    static constexpr int MAX_ANTIALIASING = 3;
    static constexpr int DEFAULT_ANTIALIASING = 3;
    IntParam antialiasing_quality =
        createIntParam("antialiasing_quality",
                       "the quality determines how many samples are used to do antialiasing, more means better quality "
                       "but increases render time (RESTART REQUIRED)",
                       luhviz_group, DEFAULT_ANTIALIASING, MIN_ANTIALIASING, MAX_ANTIALIASING);

    // viewport multiple enable/disable
    BoolParam enable_multiple_windows =
        createBoolParam("enable_multiple_windows", "Restart required. Allows floating windows. May cause some bugs.",
                        luhviz_group, true);

    // render text on top option
    BoolParam render_text_on_top = createBoolParam(
        "render_text_on_top", "If active, the text is visually always on top (not occluded by anything).", luhviz_group,
        true);

    BoolParam enable_vsync = createBoolParam(
        "enable_vsync", "Restart required. Syncs the rendering with the display update rate (typical: 60Hz)",
        luhviz_group, true);

    DoubleParam zero_kick_delay = createDoubleParam(
        "zero_kick_delay", "this delay defines the time until the kick-command is reset (in controller mode)",
        luhviz_group, 0.25, 0.0, 2);

    IntParam game_mode_width =
        createIntParam("game_mode_width", "Width of the window in game size", luhviz_group, 1920, 200, 3840);

    IntParam game_mode_height =
        createIntParam("game_mode_height", "Height of the window in game size", luhviz_group, 1080 * 2 - 80, 200, 2160);
};

}  // namespace luhsoccer::config_provider