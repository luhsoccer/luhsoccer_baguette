#include "include/info_display.hpp"

namespace luhsoccer::luhviz {

void InfoDisplay::init() {}

void InfoDisplay::render(std::unordered_map<std::string, std::unordered_map<size_t, marker::Info>>& infos) {
    const ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoNavFocus |
                                          ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollbar |
                                          ImGuiWindowFlags_NoScrollWithMouse;

    ImGui::PushStyleColor(ImGuiCol_Text, this->proxy.accent_text_color);
    ImGui::Begin("Info Display", nullptr, window_flags);
    ImGui::PopStyleColor();

    float width = ImGui::GetWindowSize().x;
    float height = ImGui::GetWindowSize().y;
    const float padding = 5.0f;

    const ImVec2 treeview_size = {width, height};
    ImGui::BeginChild("treeview", treeview_size);
    {
        for (const auto& info : infos) {
            const std::string& ns = info.first;
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + padding);
            if (ImGui::TreeNodeEx(ns.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
                for (auto& m : info.second) {
                    std::string s = m.second.getInfoText() + ": " + m.second.getInfoValue();
                    ImGui::Text("%s", s.c_str());
                }

                ImGui::TreePop();
            }

            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + padding);

            ImGui::Separator();
        }
    }

    ImGui::EndChild();

    ImGui::End();
}

}  // namespace luhsoccer::luhviz