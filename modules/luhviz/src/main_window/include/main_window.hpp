#pragma once
#include "include/data_proxy.hpp"
#include <iostream>
#include "imgui.h"
#include "time/time.hpp"
#include "iomanip"
#include "luhviz/luhviz.hpp"
#include "common/include/utils.hpp"
#include "new_rendering/include/gl_texture.hpp"

namespace luhsoccer::luhviz {

class MainWindow {
   public:
    // ----- members -----
    // ----- methods -----
    MainWindow(DataProxy& proxy) : proxy(proxy){};
    void init();
    void render(int fps, bool& parameter_settings_open, bool& skill_wizard_open, bool& robot_controller_open,
                bool& gc_window_open);

   private:
    // ----- members -----
    luhsoccer::logger::Logger logger{"luhviz/main_window"};
    const std::string luhviz_internal_config_name = "luhviz_internal";

    ImGuiViewport* viewport{};
    int fps{0};
    static constexpr float BOTTOM_BAR_HEIGHT = 80;

    bool log_playing = false;

    // icon paths
    const std::string icon_rewind = "res/images/player_icons/player_rewind.png";
    const std::string icon_play = "res/images/player_icons/player_play.png";
    const std::string icon_pause = "res/images/player_icons/player_pause.png";
    const std::string icon_stop = "res/images/player_icons/player_stop.png";
    const std::string icon_controller_connected = "res/images/status_icons/controler_connected.png";
    const std::string icon_controller_disconnected = "res/images/status_icons/controler_disconnected.png";
    const std::string icon_controller_input = "res/images/status_icons/controler_input.png";

    GLTexture gamepad_disconnected_icon;
    GLTexture gamepad_connected_icon;
    GLTexture gamepad_input_icon;
    GLTexture play_icon;
    GLTexture stop_icon;
    GLTexture pause_icon;
    GLTexture rewind_icon;

    const std::vector<std::string> vision_items = {"Disabled", "Game-Log", "Network", "Simulation"};
    const std::vector<std::string> robot_conn_items = {"Disabled",        "Network",    "Serial",
                                                       "Serial (Legacy)", "Simulation", "Simulation (Legacy)"};
    const std::vector<std::string> sim_conn_items = {"None", "Test-Simulation-Connector",
                                                     "ErForce-Simulation-Connector"};
    const std::vector<std::string> vis_upstream_items = {"Disabled", "Network"};
    const std::vector<std::string> gamecontroller_items = {"Disabled", "Network", "Internal", "Game-Log"};

    DataProxy& proxy;

    // ----- methods -----
    /**
     * @brief renders the menu bar at the top
     *
     */
    void renderMenu();

    /**
     * @brief renders the toolbar with the different tool buttons
     *
     * @param parameter_settings_open
     * @param skill_wizard_open
     * @param robot_controller_open
     */
    void renderToolbar(bool& parameter_settings_open, bool& skill_wizard_open, bool& robot_controller_open,
                       bool& gc_window_open);

    /**
     * @brief renders the info bar at the bottom with fps info and source selectors
     *
     * @param robot_controller_open
     */
    void renderBottomBar(bool& robot_controller_open);

    /**
     * @brief renders the source selector
     *
     */
    void renderSourceSelector();

    /**
     * @brief renders the robot connection selector
     *
     */
    void renderRobotConnectionSelector();

    /**
     * @brief renders the gamelog data selector
     *
     */
    void renderGamelogDataSelector();

    /**
     * @brief renders the vision upstream selector
     *
     */
    void renderVisionUpstreamSelector();

    /**
     * @brief renders the simulation connection selector
     *
     */
    void renderSimulationConnectorSelector();

    /**
     * @brief updates the user input (checks for shortcut keys pressed)
     *
     */
    void updateInput();
};
}  // namespace luhsoccer::luhviz