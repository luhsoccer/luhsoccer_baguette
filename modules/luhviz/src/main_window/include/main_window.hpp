#pragma once
#include "include/data_proxy.hpp"
#include "imgui.h"
#include "main_window/include/window_layout_handler.hpp"
#include "time/time.hpp"
#include "iomanip"
#include "luhviz/luhviz.hpp"
#include "common/include/utils.hpp"
#include "new_rendering/include/gl_texture.hpp"

struct GLFWwindow;
namespace luhsoccer::luhviz {

class MainWindow {
   public:
    // ----- members -----
    // ----- methods -----
    MainWindow(DataProxy& proxy, GLFWwindow* window) : proxy(proxy), window(window){};
    void init();
    bool render(int fps, bool& parameter_settings_open, bool& skill_wizard_open, bool& robot_controller_open,
                bool& fullscreen);

    WindowLayoutHandler& getWindowLayoutHandler();

   private:
    // ----- members -----
    luhsoccer::logger::Logger logger{"luhviz/main_window"};
    const std::string luhviz_internal_config_name = "luhviz_internal";

    WindowLayoutHandler layout_handler{};

    ImGuiViewport* viewport{};
    GLFWwindow* window;
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

    const std::vector<std::string> source_presets = {"Simulation", "Real", "Custom"};

    const std::vector<std::string> vision_items = {"Disabled", "Game-Log", "Network", "Simulation"};
    const std::vector<std::string> robot_conn_items = {"Disabled", "Network", "Serial", "Simulation"};
    const std::vector<std::string> sim_conn_items = {"None", "Test-Simulation", "ErForce-Simulation", "ErSim"};
    const std::vector<std::string> vis_upstream_items = {"Disabled", "Network"};
    const std::vector<std::string> gamecontroller_items = {"Disabled", "Network", "Internal", "Game-Log"};
    DataProxy& proxy;

    // ----- methods -----
    /**
     * @brief renders the menu bar at the top
     *
     */
    bool renderMenu(bool& fullscreen);

    /**
     * @brief renders the toolbar with the different tool buttons
     *
     * @param parameter_settings_open
     * @param skill_wizard_open
     * @param robot_controller_open
     */
    void renderToolbar(bool& parameter_settings_open, bool& skill_wizard_open, bool& robot_controller_open,
                       bool& fullscreen);

    /**
     * @brief renders the info bar at the bottom with fps info and source selectors
     */
    void renderBottomBar();

    /**
     * @brief renders the dropdown to select a source preset
     *
     */
    int renderSourcePresetSelector();

    /**
     * @brief renders the source selector
     *
     */
    void renderSourceSelector(bool custom);

    /**
     * @brief renders the robot connection selector
     *
     */
    void renderRobotConnectionSelector(bool custom);

    /**
     * @brief renders the gamelog data selector
     *
     */
    void renderGameControllerDataSelector(bool custom);

    /**
     * @brief renders the vision upstream selector
     *
     */
    void renderVisionUpstreamSelector(bool custom);

    /**
     * @brief renders the simulation connection selector
     *
     */
    void renderSimulationConnectorSelector(bool custom);

    /**
     * @brief updates the user input (checks for shortcut keys pressed)
     *
     */
    void updateInput(bool& fullscreen);
};
}  // namespace luhsoccer::luhviz