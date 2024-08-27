#include "include/info_display.hpp"

namespace luhsoccer::luhviz {

void InfoDisplay::init() {}

void InfoDisplay::render(std::map<std::string, std::map<std::string, marker::Info>>& infos, bool& open) {
    if (!open) {
        return;
    }

    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;

    ImGui::PushStyleColor(ImGuiCol_Text, this->proxy.accent_text_color);
    ImGui::Begin("Info Display", &open, window_flags);
    ImGui::PopStyleColor();

    float width = ImGui::GetWindowSize().x;
    float height = ImGui::GetWindowSize().y;
    const float padding = 5.0f;

    const ImVec2 treeview_size = {width, height};
    ImGui::BeginChild("treeview", treeview_size);
    {
        for (auto& info : infos) {
            const auto& ns = info.first;
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + padding);
            if (ImGui::TreeNodeEx(ns.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
                for (auto& m : info.second) {
                    std::string s = m.first + ": " + m.second.getInfoValue();
                    ImGui::TextColored(ImVec4(m.second.getColor().red, m.second.getColor().green,
                                              m.second.getColor().blue, m.second.getColor().alpha),
                                       "%s", s.c_str());
                }

                ImGui::TreePop();
            }
        }

        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + padding);

        ImGui::Separator();
    }

    ImGui::EndChild();

    ImGui::End();
}

}  // namespace luhsoccer::luhviz