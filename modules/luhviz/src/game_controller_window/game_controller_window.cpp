#include "include/game_controller_window.hpp"
#include "portable-file-dialogs.h"

namespace luhsoccer::luhviz {

void GameControllerWindow::init() {
    this->gc_path = config_provider::ConfigProvider::getConfigStore().luhviz_internal_config.gc_path;
    this->gc_interface = std::make_unique<game_controller_interface::GameControllerInterface>(gc_path);
}

void GameControllerWindow::render(bool* open) {
    if (!*open) {
        return;
    }

    ImGui::PushStyleColor(ImGuiCol_Text, this->proxy.accent_text_color);
    ImGui::Begin("Game Controller", open, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::PopStyleColor();

    ImGui::BeginChild("child", ImVec2{600, 150});

    ImGui::SetCursorPos(ImGui::GetCursorPos() + ImVec2{10, 10});
    ImGui::Text("GC Path: ");
    ImGui::SameLine();
    ImGui::InputText("##GC Path", &this->gc_path);
    ImGui::SameLine();
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + btn_offset);
    if (ImGui::Button("Choose Path")) {
        auto selection = pfd::open_file("Choose GC executable path", "empty").result();
        if (!selection.empty()) {
            this->gc_path = selection.front();
            this->gc_interface = std::make_unique<game_controller_interface::GameControllerInterface>(gc_path);
            config_provider::ConfigProvider::getConfigStore().luhviz_internal_config.gc_path.set(gc_path);
        }
    }

    ImGui::SetCursorPos(ImGui::GetCursorPos() + start_btn_offset);
    if (proxy.getGameControllerRunning()) {
        if (ImGui::Button(stop_str.c_str())) {
            bool success = this->gc_interface->stopGameController();
            if (success) {
                LOG_INFO(logger, "Game controller stopped");
            } else {
                LOG_WARNING(logger, "Error while stopping game controller");
            }
            proxy.getGameControllerRunning() = !success;
        }
    } else {
        if (ImGui::Button(start_str.c_str())) {
            bool success = this->gc_interface->startGameController();
            if (success) {
                LOG_INFO(logger, "Game controller running on localhost:8081");
            } else {
                LOG_WARNING(logger, "Error while starting game controller");
            }
            proxy.getGameControllerRunning() = success;
        }
    }
    ImGui::EndChild();
    ImGui::End();
}

}  // namespace luhsoccer::luhviz