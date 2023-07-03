#pragma once

#include "config_provider/parameters.hpp"
#include "config_provider/config_base.hpp"

namespace luhsoccer::config_provider {

struct LuhvizInternalConfig : public Config {
    LuhvizInternalConfig() : Config("luhviz_internal.toml", datatypes::ConfigType::LOCAL) {}

    const std::string luhviz_group = "luhviz_internal";

    static constexpr int DEFAULT_WINDOW_WIDTH = 1280;
    static constexpr int DEFAULT_WINDOW_HEIGHT = 640;
    IntParam window_width = createIntParam("window_width", "the default window width", luhviz_group,
                                           DEFAULT_WINDOW_WIDTH, 0, std::numeric_limits<int>::max());
    IntParam window_height = createIntParam("window_height", "the default window height", luhviz_group,
                                            DEFAULT_WINDOW_HEIGHT, 0, std::numeric_limits<int>::max());

    static constexpr float PI = L_PI;
    static constexpr float DEFAULT_ANGLE_X = PI;
    static constexpr float DEFAULT_ANGLE_Y = PI / 4;
    static constexpr float DEFAULT_ANGLE_ORTH_Y = PI * 2;
    static constexpr float DEFAULT_ZOOM = 15.0f;
    static constexpr float MIN_ZOOM = 1.0f;
    static constexpr float MAX_ZOOM = 35.0f;
    static constexpr bool DEFAULT_PERSPECTIVE = true;
    BoolParam perspective_view =
        createBoolParam("perspective_view", "the default view mode", luhviz_group, DEFAULT_PERSPECTIVE);
    DoubleParam camera_orientation_angle_x =
        createDoubleParam("camera_angle_x", "default camera x angle", luhviz_group, DEFAULT_ANGLE_X, 0, 2 * PI);
    DoubleParam camera_orientation_angle_y =
        createDoubleParam("camera_angle_y", "default camera y angle", luhviz_group, DEFAULT_ANGLE_Y, 0, PI / 2);
    DoubleParam camera_orientation_zoom =
        createDoubleParam("camera_zoom", "default camera zoom", luhviz_group, DEFAULT_ZOOM, MIN_ZOOM, MAX_ZOOM);

    StringParam disabled_namespaces =
        createStringParam("disabled_namespaces", "the disabled inspector namespaces", luhviz_group, "");

    // selected bottom sources
    IntParam selected_vision_source =
        createIntParam("selected_vision_source", "saves the selected source", luhviz_group, 0);
    IntParam selected_simulation_connector =
        createIntParam("selected_simulation_connector", "saves the selected source", luhviz_group, 0);
    IntParam selected_robot_connector =
        createIntParam("selected_robot_connector", "saves the selected source", luhviz_group, 0);
    IntParam selected_vision_upstream =
        createIntParam("selected_vision_upstream", "saves the selected source", luhviz_group, 0);
    IntParam selected_gamelog_source =
        createIntParam("selected_gamelog_source", "saves the selected source", luhviz_group, 0);
    DoubleParam selected_controler_velocity =
        createDoubleParam("selected_controler_velocity", "saves the movement velocity", luhviz_group, 1.0f, 0.0f, 5.0f);

    // Debugger Options
    const std::string luhviz_debugger = "luhviz-debugger";
    BoolParam autoscroll = createBoolParam("autoscroll", "autoscroll", luhviz_debugger, true);
    BoolParam show_log_type = createBoolParam("show_log_type", "display log type", luhviz_debugger, true);
    BoolParam show_timestamps = createBoolParam("show_timestamps", "timestamps", luhviz_debugger, true);
    BoolParam show_milliseconds = createBoolParam("show_milliseconds", "display milliseconds", luhviz_debugger, true);
    BoolParam show_module = createBoolParam("show_module", "display source module", luhviz_debugger, true);
    BoolParam show_location = createBoolParam("show_location", "display source file and line", luhviz_debugger, true);
    BoolParam word_wrap_text =
        createBoolParam("word_wrap_text", "dynamically word wrap text in the console", luhviz_debugger, false);
    BoolParam use_monospace_font =
        createBoolParam("use_monospace_font", "use a monospace font to display console logs", luhviz_debugger, false);
    BoolParam add_extra_spacing_between_logs = createBoolParam(
        "add_extra_spacing_between_logs", "add extra spacing between logs in the console", luhviz_debugger, false);

    BoolParam baguette_mode = createBoolParam("baguette_mode", "enable/disable baguette mode", luhviz_debugger, false);

    BoolParam show_trace = createBoolParam("show_trace", "display trace logs for ALL modules", luhviz_debugger, true);
    BoolParam show_debug = createBoolParam("show_debug", "display debug logs for ALL modules", luhviz_debugger, true);
    BoolParam show_info = createBoolParam("show_info", "display info logs for ALL modules", luhviz_debugger, true);
    BoolParam show_warning =
        createBoolParam("show_warning", "display warning logs for ALL modules", luhviz_debugger, true);
    BoolParam show_error = createBoolParam("show_error", "display error logs for ALL modules", luhviz_debugger, true);
    BoolParam show_critical =
        createBoolParam("show_critical", "display critical logs for ALL modules", luhviz_debugger, true);
    BoolParam show_off = createBoolParam("show_off", "display off logs for ALL modules", luhviz_debugger, true);
    BoolParam show_other = createBoolParam("show_other", "display other logs for ALL modules", luhviz_debugger, true);

    StringParam module_filters = createStringParam(
        "module_filters", "stores module filter settings for each individual module", luhviz_debugger, "");

    // GC executable path
    StringParam gc_path =
        createStringParam("gc_path", "path to the executable of the game controller", luhviz_group, "empty");
};
}  // namespace luhsoccer::config_provider