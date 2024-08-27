#pragma once

#include "debugger/include/debugger_event.hpp"
#include "debugger/include/module_filter.hpp"
#include "debugger/include/debugger_event.hpp"
#include "debugger/include/module_filter.hpp"
#include "include/data_proxy.hpp"
#include "common/include/fonts.hpp"
#include "common/include/fonts.hpp"

#include <utility>
#include <tuple>
#include "logger/logger.hpp"
#include "logger/gui_callback.hpp"
#include "include/data_proxy.hpp"

namespace luhsoccer::luhviz {
class Debugger {
   public:
    explicit Debugger(Fonts& fonts, DataProxy& proxy);
    void init();
    void render(bool& open);
    void drawWindow(ImGuiWindowFlags& flags, bool& open);
    void drawConsole();
    void addEvent(const DebuggerEvent& event);
    void addEventFromLogger(const logger::LoggerDetails& log);
    void printEventAsInputText(DebuggerEvent& event, int index);
    void printEventAsTextColored(DebuggerEvent& event, int index);
    void printEvents(bool new_logs_available);
    void printEventsRewritten(bool new_logs_available);
    [[nodiscard]] std::string formatEventToString(const DebuggerEvent& event) const;
    void wrapText(DebuggerEvent& event);
    void baguetteMode();

    // This method should be called when changes to the all modules log filter log are made so that they are saved in
    // the config
    void updateAllModulesLogTypeConfig();

    // Loads all stored option configurations
    void loadConfigs();

    // Saves a single module's log type filters in the config
    // Module filters are stored in the following format:
    // "module_name_1-tracebool-debugbool-...-otherbool|module_name_2-tracebool-debugbool-...-otherbool|" etc.
    // The booleans are converted to strings (0 for false, 1 for true)
    // and they represent the ModuleFilter data for each log type
    void updateModuleFilter(const std::string& module_name, ModuleFilter* module_filter);

   private:
    DataProxy& proxy;  // Used to save/load saved config values

    const std::string title = "Console";  // The (constant) title of the window

    std::vector<DebuggerEvent> event_log;           // Stores all DebuggerEvents
    std::vector<DebuggerEvent> filtered_event_log;  // Stores DebuggerEvents that fit the current filter criteria

    std::vector<std::string> module_list = {"All modules"};  // Stores all modules that have logged an event so far

    std::map<std::string, ModuleFilter> module_filter_map;  // Stores all modules that have logged an event so
                                                            // far, as well as the enabled log types for each

    // std::vector<float> event_heights;  // Stores the height in pixels of each stored event (used for the Clipper)

    static constexpr long unsigned int MAX_LOG_SIZE = 1000;  // Stores the limit for the max number of logs stored (to
                                                             // improve performance). If set to 0, there is no limit

    bool autoscroll = true;         // Keeps track of whether autoscroll is enabled or disabled
    bool show_log_type = true;      // Keeps track of whether log types are enabled or disabled
    bool show_timestamps = true;    // Keeps track of whether timestamps are enabled or disabled
    bool show_milliseconds = true;  // Keeps track of whether timestamps are enabled or disabled
    bool show_module = true;        // Keeps track of whether showing the source module is enabled or disabled
    bool show_location = true;      // Keeps track of whether showing the source file and line is enabled or disabled

    bool show_trace = true;     // Keeps track of whether trace logs are enabled or disabled
    bool show_debug = true;     // Keeps track of whether debug logs are enabled or disabled
    bool show_info = true;      // Keeps track of whether info logs are enabled or disabled
    bool show_warning = true;   // Keeps track of whether warning logs are enabled or disabled
    bool show_error = true;     // Keeps track of whether error logs are enabled or disabled
    bool show_critical = true;  // Keeps track of whether critical logs are enabled or disabled
    bool show_off = true;       // Keeps track of whether off logs are enabled or disabled
    bool show_other = true;     // Keeps track of whether other logs are enabled or disabled

    bool word_wrap_text = false;                  // Console logs will be dynamically word-wrapped to fit in the width
    bool use_monospace_font = false;              // A monospace font will be used for the console if true
    bool add_extra_spacing_between_logs = false;  // Adds extra spacing between log entries if true

    bool baguette_mode = false;               // Is baguette mode active?
    unsigned int baguette_mode_frame = 0;     // Current baguette mode frame number
    bool baguette_mode_frame_advance = true;  // Baguette mode will only advance its frames if this is true
    std::vector<std::string> baguette_mode_frames{"", "", "", "",
                                                  "", "", "", ""};  // Baguette mode stores each frame as a string here
    const float baguette_mode_color_increment =
        0.1f;                                 // Baguette mode's color values will be increased/decreased by this much
    float baguette_mode_color_red = 1.00f;    // Keeps track of baguette mode's red value
    float baguette_mode_color_green = 0.00f;  // Keeps track of baguette mode's green value
    float baguette_mode_color_blue = 0.00f;   // Keeps track of baguette mode's blue value
    unsigned int baguette_mode_color_phase = 0;  // Keeps track of baguette mode's current color phase
    // Phase 0: 255 0 0 -> 255 255 0
    // Phase 1: 255 255 0 -> 0 255 0
    // Phase 2: 0 255 0 -> 0 255 255
    // Phase 3: 0 255 255 -> 0 0 255
    // Phase 4: 0 0 255 -> 255 0 255
    // Phase 5: 255 0 255 -> 255 0 0

    std::string filter;             // Stores active search filters
    int events_shown = 0;           // Keeps track of how many events fulfill the search/filter criteria
    int previous_events_shown = 0;  // Keeps track of how many events fulfilled the search/filter criteria in the
                                    // previous frame (used to calculate if new logs are available)

    Fonts& fonts;  // Reference to the stored fonts of Luhviz

    bool previous_monospace_setting =
        false;  // Stores whether the monospace font was on or off during the previous print cycle,
    // we need this to know if we need to rewrap the text because the user changed the font

    static constexpr float EXTRA_SPACING =
        6.0f;  // The extra spacing to be used between logs (if the relevant option is enabled)

    const std::string config_name = "luhviz_internal";
};

}  // namespace luhsoccer::luhviz