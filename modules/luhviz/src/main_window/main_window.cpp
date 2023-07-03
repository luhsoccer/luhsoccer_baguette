#include "include/main_window.hpp"
#include "portable-file-dialogs.h"

namespace luhsoccer::luhviz {

void MainWindow::init() {
    // setup icons for gamepad
    this->gamepad_disconnected_icon.create(icon_controller_disconnected, true, true);
    this->gamepad_connected_icon.create(icon_controller_connected, true, true);
    this->gamepad_input_icon.create(icon_controller_input, true, true);
    this->play_icon.create(icon_play, false, true);
    this->pause_icon.create(icon_pause, false, true);
    this->stop_icon.create(icon_stop, false, true);
    this->rewind_icon.create(icon_rewind, false, true);

    // set saved sources

    // vision source
    int vision_index = this->proxy.getConfigInt(luhviz_internal_config_name, "selected_vision_source");
    if (vision_index < static_cast<int>(this->vision_items.size())) {
        this->proxy.setVisionSource(this->vision_items[vision_index]);
    }
    // robot connector source
    int robot_conn_index = this->proxy.getConfigInt(luhviz_internal_config_name, "selected_robot_connector");
    if (robot_conn_index < static_cast<int>(this->robot_conn_items.size())) {
        this->proxy.setRobotConnection(this->robot_conn_items[robot_conn_index]);
    }
    // sim connectionn source
    int sim_conn_index = this->proxy.getConfigInt(luhviz_internal_config_name, "selected_simulation_connector");
    if (sim_conn_index < static_cast<int>(this->sim_conn_items.size())) {
        this->proxy.setSimulationConnection(this->sim_conn_items[sim_conn_index]);
    }
    // vision upstream source
    int vis_upstream_index = this->proxy.getConfigInt(luhviz_internal_config_name, "selected_vision_upstream");
    if (vis_upstream_index < static_cast<int>(this->vis_upstream_items.size())) {
        this->proxy.setVisionPublishMode(this->vis_upstream_items[vis_upstream_index]);
    }
    // gamecontroller source
    int gamecontroller_index = this->proxy.getConfigInt(luhviz_internal_config_name, "selected_gamelog_source");
    if (gamecontroller_index < static_cast<int>(this->gamecontroller_items.size())) {
        this->proxy.setGameControllerDataSource(this->gamecontroller_items[gamecontroller_index]);
    }
}

void MainWindow::render(int fps, bool& parameter_settings_open, bool& skill_wizard_open, bool& robot_controller_open,
                        bool& gc_window_open) {
    this->fps = fps;

    viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->Size);
    ImGui::SetNextWindowViewport(viewport->ID);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_DockingEmptyBg, ImVec4(0, 0, 0, 0));
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar |
                                    ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                                    ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoBringToFrontOnFocus |
                                    ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;

    ImGui::Begin("MainWindow", nullptr, window_flags);

    // Menu
    renderMenu();

    // Toolbar
    renderToolbar(parameter_settings_open, skill_wizard_open, robot_controller_open, gc_window_open);
    ImGui::DockSpace(ImGui::GetID("MainDockSpace"), ImVec2(viewport->Size.x, viewport->Size.y - BOTTOM_BAR_HEIGHT));

    // Bottombar
    renderBottomBar(robot_controller_open);

    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleColor();
}

void MainWindow::renderMenu() {
    constexpr float MARGIN = 5;
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("Shortcuts")) {
            ImGui::Spacing();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + MARGIN);
            if (ImGui::MenuItem("Robot de-/selection", "Left click on robot", false, false)) {
            }
            ImGui::Spacing();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + MARGIN);
            if (ImGui::MenuItem("Related robot de-/selection", "CTRL + left click on robot", false, false)) {
            }
            ImGui::Separator();
            ImGui::Spacing();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + MARGIN);
            if (ImGui::MenuItem("Teleport robot", "t", false, false)) {
            }
            ImGui::Spacing();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + MARGIN);
            if (ImGui::MenuItem("Teleport ball", "z", false, false)) {
            }
            ImGui::Spacing();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + MARGIN);
            if (ImGui::MenuItem("set point/execute skill", "x", false, false)) {
            }
            ImGui::Spacing();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + MARGIN);
            if (ImGui::MenuItem("Cancel teleport/skill", "ESC", false, false)) {
            }
            ImGui::Separator();
            ImGui::Spacing();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + MARGIN);
            if (ImGui::MenuItem("Open/close Config", "p", false, false)) {
            }
            ImGui::Spacing();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + MARGIN);
            if (ImGui::MenuItem("Close luhviz", "CTRL + q", false, false)) {
            }
            ImGui::Spacing();
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Game-Log Player")) {
            ImGui::Spacing();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + MARGIN);
            if (ImGui::MenuItem("Load SSL-Log")) {
                auto selection = pfd::open_file("Load SSL Game-Log", "").result();
                if (!selection.empty()) {
                    this->proxy.setLogFilePath(selection.front());
                    LOG_INFO(logger, "Loaded SSL Logfile: {}", selection.front());
                }
            }
            ImGui::Spacing();
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + MARGIN);
            if (ImGui::MenuItem("Load Luhviz-Log")) {
                auto selection = pfd::open_file("Load Luhviz Game-Log", "", {".luhviz"}).result();
                if (!selection.empty()) {
                    this->proxy.setLogFilePath(selection.front());
                    LOG_INFO(logger, "Loaded SSL Logfile: {}", selection.front());
                }
            }
            ImGui::Spacing();
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }
}

void MainWindow::renderToolbar(bool& parameter_settings_open, bool& skill_wizard_open, bool& robot_controller_open,
                               bool& gc_window_open) {
    ImGuiIO& io = ImGui::GetIO();

    ImGui::SameLine();
    constexpr int OFFSET = 5;
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + OFFSET);
    if (ImGui::Button("Parameter Settings") || (ImGui::IsKeyPressed(ImGuiKey_P) && !io.WantTextInput)) {
        parameter_settings_open = !parameter_settings_open;
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", "Open the Parameter Configuration.\nShortcut: 'p' ");

    ImGui::SameLine();
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + OFFSET);
    if (ImGui::Button("Skill Wizard")) {
        skill_wizard_open = !skill_wizard_open;
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", "Open the Skill Wizard to change or add skills.");

    ImGui::SameLine();
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + OFFSET);
    if (ImGui::Button("Gamepad controller")) {
        robot_controller_open = !robot_controller_open;
    }
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", "Open the Gamepad controller to steer a robot with the gamepad.");

    ImGui::SameLine();
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + OFFSET);
    if (ImGui::Button("GC Window")) {
        gc_window_open = !gc_window_open;
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", "Open the GC window to start/stop the game controller.");

    // Renderview action Buttons
    updateInput();

    // SSL Log Player Controls
    constexpr ImVec2 BUTTON_SIZE{16, 16};
    constexpr int X_OFFSET = 30;
    ImGui::SameLine();
    ImGui::SetCursorPos({ImGui::GetCursorPosX() + X_OFFSET, ImGui::GetCursorPosY() + OFFSET});
    ImGui::Text("Game-Log player: ");
    ImGui::SameLine();
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + OFFSET);
    if (ImGui::ImageButton(this->rewind_icon.getImguiId(), BUTTON_SIZE)) {
    }
    GLTexture& play_pause_icon = this->log_playing ? this->pause_icon : this->play_icon;
    ImGui::SameLine();
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + OFFSET);
    if (ImGui::ImageButton(play_pause_icon.getImguiId(), BUTTON_SIZE)) {
        this->log_playing = !this->log_playing;
    }
    ImGui::SameLine();
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + OFFSET);
    if (ImGui::ImageButton(this->stop_icon.getImguiId(), BUTTON_SIZE)) {
    }

    // display file name
    if (this->proxy.getLogFilePath().has_value()) {
        ImGui::SameLine();
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + OFFSET);
        std::string loaded_file = "File: " + this->proxy.getLogFilePath().value();
        ImGui::Text(loaded_file.c_str());
    }

    ImGui::Separator();
}

void MainWindow::renderSourceSelector() {
    int saved_index = this->proxy.getConfigInt(luhviz_internal_config_name, "selected_vision_source");
    int default_index = saved_index >= static_cast<int>(vision_items.size()) ? 0 : saved_index;
    static std::string current_vision_source = vision_items[default_index];

    auto it = find(vision_items.begin(), vision_items.end(), this->proxy.getVisionSource());
    if (it != vision_items.end()) {
        // calculating the index
        int index = static_cast<int>(it - vision_items.begin());
        current_vision_source = vision_items[index];

        ImGui::TextColored(proxy.accent_text_color, "Vision Source:");
        ImGui::SameLine();

        const float width = Utils::getMaxItemSize(vision_items, 30).x;
        ImGui::SetNextItemWidth(width);
        if (ImGui::BeginCombo("##VS", current_vision_source.c_str())) {
            for (const auto& item : vision_items) {
                bool is_selected = (current_vision_source == item);
                if (ImGui::Selectable(item.c_str(), is_selected)) current_vision_source = item;
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();

            proxy.setVisionSource(current_vision_source);
        }
        this->proxy.setConfigInt("selected_vision_source", index);
    }
}

void MainWindow::renderRobotConnectionSelector() {
    int saved_index = this->proxy.getConfigInt(luhviz_internal_config_name, "selected_robot_connector");
    int default_index = saved_index >= static_cast<int>(robot_conn_items.size()) ? 0 : saved_index;
    static std::string current_robot_conn = robot_conn_items[default_index];

    auto it = find(robot_conn_items.begin(), robot_conn_items.end(), this->proxy.getRobotConnection());
    if (it != robot_conn_items.end()) {
        // calculating the index
        int index = static_cast<int>(it - robot_conn_items.begin());
        current_robot_conn = robot_conn_items[index];

        ImGui::SameLine();
        ImGui::TextColored(proxy.accent_text_color, "Robot Connector:");
        ImGui::SameLine();

        const float width = Utils::getMaxItemSize(robot_conn_items, 30).x;
        ImGui::SetNextItemWidth(width);
        if (ImGui::BeginCombo("##RC", current_robot_conn.c_str())) {
            for (const auto& item : robot_conn_items) {
                bool is_selected = (current_robot_conn == item);
                if (ImGui::Selectable(item.c_str(), is_selected)) current_robot_conn = item;
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();

            proxy.setRobotConnection(current_robot_conn);
        }
        this->proxy.setConfigInt("selected_robot_connector", index);
    }
}

void MainWindow::renderSimulationConnectorSelector() {
    int saved_index = this->proxy.getConfigInt(luhviz_internal_config_name, "selected_simulation_connector");
    int default_index = saved_index >= static_cast<int>(sim_conn_items.size()) ? 0 : saved_index;
    static std::string current_sim_connector = sim_conn_items[default_index];

    auto it = find(sim_conn_items.begin(), sim_conn_items.end(), this->proxy.getSimulationConnection());
    if (it != sim_conn_items.end()) {
        // calculating the index
        int index = static_cast<int>(it - sim_conn_items.begin());
        current_sim_connector = sim_conn_items[index];

        ImGui::SameLine();
        ImGui::TextColored(proxy.accent_text_color, "Simulation Connector:");
        ImGui::SameLine();

        const float width = Utils::getMaxItemSize(sim_conn_items, 30).x;
        ImGui::SetNextItemWidth(width);
        if (ImGui::BeginCombo("##SIM", current_sim_connector.c_str())) {
            for (const auto& item : sim_conn_items) {
                bool is_selected = (current_sim_connector == item);
                if (ImGui::Selectable(item.c_str(), is_selected)) current_sim_connector = item;
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();

            proxy.setSimulationConnection(current_sim_connector);
        }
        this->proxy.setConfigInt("selected_simulation_connector", index);
    }
}

void MainWindow::renderVisionUpstreamSelector() {
    int saved_index = this->proxy.getConfigInt(luhviz_internal_config_name, "selected_vision_upstream");
    int default_index = saved_index >= static_cast<int>(vis_upstream_items.size()) ? 0 : saved_index;
    static std::string current_upstream_mode = vis_upstream_items[default_index];

    auto it = find(vis_upstream_items.begin(), vis_upstream_items.end(), this->proxy.getVisionPublishMode());
    if (it != vis_upstream_items.end()) {
        // calculating the index
        int index = static_cast<int>(it - vis_upstream_items.begin());
        current_upstream_mode = vis_upstream_items[index];

        ImGui::SameLine();
        ImGui::TextColored(proxy.accent_text_color, "Vision Upstream:");
        ImGui::SameLine();

        const float width = Utils::getMaxItemSize(vis_upstream_items, 30).x;
        ImGui::SetNextItemWidth(width);
        if (ImGui::BeginCombo("##SC", current_upstream_mode.c_str())) {
            for (const auto& item : vis_upstream_items) {
                bool is_selected = (current_upstream_mode == item);
                if (ImGui::Selectable(item.c_str(), is_selected)) current_upstream_mode = item;
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();

            proxy.setVisionPublishMode(current_upstream_mode);
        }
        this->proxy.setConfigInt("selected_vision_upstream", index);
    }
}

void MainWindow::renderGamelogDataSelector() {
    int saved_index = this->proxy.getConfigInt(luhviz_internal_config_name, "selected_gamelog_source");
    int default_index = saved_index >= static_cast<int>(gamecontroller_items.size()) ? 0 : saved_index;
    static std::string current_gamedata_source = gamecontroller_items[default_index];

    auto it = find(gamecontroller_items.begin(), gamecontroller_items.end(), this->proxy.getGameControllerDataSource());
    if (it != gamecontroller_items.end()) {
        // calculating the index
        int index = static_cast<int>(it - gamecontroller_items.begin());
        current_gamedata_source = gamecontroller_items[index];

        ImGui::SameLine();
        ImGui::TextColored(proxy.accent_text_color, "GC Selector:");
        ImGui::SameLine();

        const float width = Utils::getMaxItemSize(gamecontroller_items, 30).x;
        ImGui::SetNextItemWidth(width);
        if (ImGui::BeginCombo("##GS", current_gamedata_source.c_str())) {
            for (const auto& item : gamecontroller_items) {
                bool is_selected = (current_gamedata_source == item);
                if (ImGui::Selectable(item.c_str(), is_selected)) current_gamedata_source = item;
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();

            proxy.setGameControllerDataSource(current_gamedata_source);
        }
        this->proxy.setConfigInt("selected_gamelog_source", index);
    }
}

void MainWindow::renderBottomBar(bool& robot_controller_open) {
    constexpr ImVec4 PRIMARY_COLOR_BLUE = {0.0f, 0.4666666686534882f, 0.7843137383460999f, 1.0f};
    constexpr ImVec4 PRIMARY_COLOR_YELLOW = {1.0f, 1.0f, 0.0f, 1.0f};

    bool color_blue = config_provider::ConfigProvider::getConfigStore().game_config.is_blue;
    if (color_blue) {
        ImGui::PushStyleColor(ImGuiCol_ChildBg, PRIMARY_COLOR_BLUE);
    } else {
        ImGui::PushStyleColor(ImGuiCol_ChildBg, PRIMARY_COLOR_YELLOW);
    }

    ImGui::BeginChild("BottomBar", {viewport->Size.x, BOTTOM_BAR_HEIGHT}, true);

    renderSourceSelector();
    renderSimulationConnectorSelector();
    renderRobotConnectionSelector();
    renderVisionUpstreamSelector();
    renderGamelogDataSelector();

    std::stringstream stream;
    stream << std::fixed << std::setprecision(1) << time::timeSinceStart().asSec();
    std::string text =
        "time elapsed: " + stream.str() + "s               " + std::to_string(fps) + " Fps    Luhviz Version 0.2";
    ImVec2 size = ImGui::CalcTextSize(text.c_str());

    ImGui::SetCursorPos({viewport->Size.x - size.x - 50, ImGui::GetCursorPosY() + 2});
    ImGui::SameLine();
    ImGui::SetCursorPosX(viewport->Size.x - size.x - 50);
    ImGui::Text("%s", text.c_str());

    ImGui::SameLine();
    std::string conti_count = std::to_string(this->proxy.getGamepadCount());
    ImVec2 count_offsett = ImGui::CalcTextSize(conti_count.c_str());
    ImGui::SetCursorPosX(viewport->Size.x - 30 - count_offsett.x);

    ImGui::Text(conti_count.c_str());
    ImGui::SameLine();
    ImGui::SetCursorPos({viewport->Size.x - 30 + count_offsett.x, ImGui::GetCursorPosY() + 3});

    constexpr ImVec2 BUTTON_SIZE{20.0f, 14.54f};
    if (this->proxy.getGamepadStatus() == GamepadStatus::CONNECTED) {
        ImGui::Image(gamepad_connected_icon.getImguiId(), BUTTON_SIZE);
    } else if (this->proxy.getGamepadStatus() == GamepadStatus::INPUT_RECEIVED) {
        ImGui::Image(gamepad_input_icon.getImguiId(), BUTTON_SIZE);
    } else {
        ImGui::Image(gamepad_disconnected_icon.getImguiId(), BUTTON_SIZE);
    }

    ImGui::EndChild();
    ImGui::PopStyleColor();
}

void MainWindow::updateInput() {
    const ImGuiIO& io = ImGui::GetIO();

    // teleport function
    if (ImGui::IsKeyPressed(ImGuiKey_T) && !io.WantTextInput) {
        if (this->proxy.getManipulationMode() == ManipulationMode::TELEPORT_ROBOT) {
            this->proxy.getManipulationMode() = ManipulationMode::SELECT;
        } else {
            this->proxy.getManipulationMode() = ManipulationMode::TELEPORT_ROBOT;
        }
    }

    // teleport ball function
    else if (ImGui::IsKeyPressed(ImGuiKey_Z) && !io.WantTextInput) {
        if (this->proxy.getManipulationMode() == ManipulationMode::TELEPORT_BALL) {
            this->proxy.getManipulationMode() = ManipulationMode::SELECT;
        } else {
            this->proxy.getManipulationMode() = ManipulationMode::TELEPORT_BALL;
        }
    }

    // send skill function
    else if (ImGui::IsKeyPressed(ImGuiKey_X) && !io.WantTextInput) {
        if (this->proxy.getManipulationMode() == ManipulationMode::ADD_POINT ||
            this->proxy.getManipulationMode() == ManipulationMode::ADD_DIRECTION ||
            this->proxy.getManipulationMode() == ManipulationMode::EXECUTE_SKILL) {
            this->proxy.getManipulationMode() = ManipulationMode::SELECT;
        } else {
            if (this->proxy.getRemainingPointsToChoose() == 0) {
                this->proxy.getManipulationMode() = ManipulationMode::EXECUTE_SKILL;
            } else {
                this->proxy.getManipulationMode() = ManipulationMode::ADD_POINT;
            }
        }
    }

    else if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        this->proxy.getManipulationMode() = ManipulationMode::SELECT;
    }
}

}  // namespace luhsoccer::luhviz