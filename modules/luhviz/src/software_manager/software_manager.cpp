#include "include/software_manager.hpp"

#ifdef _WIN32
#define NOGDI
#include <Windows.h>
#else
#include <cstdlib>
#endif

namespace luhsoccer::luhviz {

void SoftwareManager::render(bool& open) {
    if (!open) {
        return;
    }

    ImGui::PushStyleColor(ImGuiCol_Text, this->proxy.accent_text_color);
    ImGui::Begin("Software-Manager", &open);
    ImGui::PopStyleColor();

    for (const auto& component : this->proxy.getSoftwareManager().getComponents()) {
        ImGui::PushID(format_as(component).data());
        ImGui::Text("%s", format_as(component).data());

        bool running = this->proxy.getSoftwareManager().isRunning(component);
        bool should_run = this->proxy.getSoftwareManager().shouldRun(component);

        ImGui::BeginDisabled(running || should_run);
        ImGui::SameLine();
        if (ImGui::Button("Start")) {
            this->proxy.getSoftwareManager().startComponent(component);
        }
        ImGui::EndDisabled();

        ImGui::BeginDisabled(!running || !should_run);
        ImGui::SameLine();
        if (ImGui::Button("Stop")) {
            this->proxy.getSoftwareManager().stopComponent(component);
        }

        if (component == software_manager::SoftwareComponent::GAME_CONTROLLER) {
            ImGui::SameLine();
            if (ImGui::Button("Open GC in Browser")) {
                // TODO find a nicer way to open the browser
                // Maybe use the utils module instead of luhviz to abstract the platform
#ifdef _WIN32
                ShellExecuteA(NULL, "open", "http://localhost:26782", NULL, NULL, SW_SHOWNORMAL);
#else
                system("xdg-open http://localhost:26782");
#endif
            }

            if (running && proxy.getGameControllerDataSource() != "Internal") {
                ImGui::Text("To use the GameController, select 'Internal' as the gc source");
            }
        } else if (component == software_manager::SoftwareComponent::ER_SIM) {
            if (running && proxy.getSimulationConnection() != "ErSim") {
                ImGui::Text("To use the ErSim simulator, select 'ErSim' as the simulation source");
            }
        }

        ImGui::EndDisabled();

        ImGui::Separator();
        ImGui::PopID();
    }

    ImGui::End();
}
}  // namespace luhsoccer::luhviz