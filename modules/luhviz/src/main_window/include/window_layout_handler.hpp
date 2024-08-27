#pragma once

#include "config_provider/config_store_main.hpp"

namespace luhsoccer::luhviz {
class WindowLayoutHandler {
   public:
    WindowLayoutHandler() = default;

    bool& getRenderViewOpen() { return render_view_open; }
    bool& getInspectorOpen() { return inspector_open; }
    bool& getConsoleOpen() { return console_open; }
    bool& getManipulatorOpen() { return manipulator_open; }
    bool& getGameInfoOpen() { return game_info_open; }
    bool& getRobertDisplayOpen() { return robert_display_open; }
    bool& getInfoDisplayOpen() { return info_display_open; }
    bool& getPlotterOpen() { return plotter_open; }
    bool& getGameLogOpen() { return game_log_open; }
    bool& getSoftwareManagerOpen() { return software_manager_open; }
    bool setFullscreen(bool fullscreen) {
        render_view_open = true;
        // change to fullscreen
        if (fullscreen == true && render_view_fullscreen == false) {
            inspector_open_before = inspector_open;
            inspector_open = false;
            console_open_before = console_open;
            console_open = false;
            manipulator_open_before = manipulator_open;
            manipulator_open = false;
            game_info_open_before = game_info_open;
            game_info_open = false;
            game_log_open_before = game_log_open;
            game_log_open = false;
            software_manager_open_before = software_manager_open;
            software_manager_open = false;
            robert_display_open_before = robert_display_open;
            robert_display_open = false;
            info_display_open_before = info_display_open;
            info_display_open = false;
            plotter_open_before = plotter_open;
            plotter_open = false;
        }
        // go back from fullscreen to normal
        else if (fullscreen == false && render_view_fullscreen == true) {
            inspector_open = inspector_open_before;
            console_open = console_open_before;
            manipulator_open = manipulator_open_before;
            game_info_open = game_info_open_before;
            game_log_open = game_log_open_before;
            software_manager_open = software_manager_open_before;
            robert_display_open = robert_display_open_before;
            info_display_open = info_display_open_before;
            plotter_open = plotter_open_before;
        }
        render_view_fullscreen = fullscreen;
        return render_view_fullscreen;
    }

    void loadLayout();

    void setDefaultLayout();

   private:
    bool render_view_open;
    bool inspector_open;
    bool console_open;
    bool manipulator_open;
    bool game_info_open;
    bool game_log_open;
    bool software_manager_open;
    bool robert_display_open;
    bool info_display_open;
    bool plotter_open;
    bool render_view_fullscreen{};

    bool inspector_open_before;
    bool console_open_before;
    bool manipulator_open_before;
    bool game_info_open_before;
    bool game_log_open_before;
    bool software_manager_open_before;
    bool robert_display_open_before;
    bool info_display_open_before;
    bool plotter_open_before;
};
}  // namespace luhsoccer::luhviz