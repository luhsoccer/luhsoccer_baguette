#include "include/robot_info_display.hpp"

namespace luhsoccer::luhviz {

void RobotInfoDisplay::init() {}

void RobotInfoDisplay::render(const std::unordered_map<std::string, marker::RobotInfo>& robot_infos) {
    ImGui::PushStyleColor(ImGuiCol_Text, this->proxy.accent_text_color);
    ImGui::Begin("Robot Display", nullptr, window_flags);
    ImGui::PopStyleColor();

    // sort robots
    this->sorted_robot_infos.clear();
    for (const auto& r_info : robot_infos) {
        this->sorted_robot_infos.emplace_back(r_info.second);
    }
    std::sort(this->sorted_robot_infos.begin(), this->sorted_robot_infos.end(),
              [](const marker::RobotInfo& info1, const marker::RobotInfo& info2) {
                  return info1.getRobotId().id < info2.getRobotId().id;
              });

    // render all robot infos as boxes
    ImVec2 coord{0, 0};
    ImVec2 pos{0, 0};
    for (const auto& r_info : this->sorted_robot_infos) {
        ImVec2 block_size = renderRobotInfoBlock(r_info, coord, pos, ImGui::GetWindowWidth());

        if (coord.x == 0 && 2 * block_size.x < ImGui::GetWindowWidth()) {
            coord.x = 1;
        } else {
            coord.x = 0;
            coord.y += 1;
        }
    }

    ImGui::End();
}

ImVec2 RobotInfoDisplay::renderRobotInfoBlock(const marker::RobotInfo& info, const ImVec2& coord, ImVec2& pos,
                                              float window_width) {
    float x_offset = pos.x;
    float y_offset = pos.y;
    ImGui::SetCursorPos(ImVec2{LEFT_PADDING + x_offset, TOP_PADDING + y_offset});
    ImGui::BeginGroup();
    // render Robot id
    ImGui::Text("Robot %zu", info.getRobotId().id);

    // render robot status label
    ImGui::SameLine();
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + STATUS_PADDING_LEFT);
    const std::string status = info.getStatus();
    ImVec2 status_size = ImGui::CalcTextSize(status.c_str());
    ImVec2 start_pos = {ImGui::GetItemRectMax().x + STATUS_BORDER_LEFT, ImGui::GetItemRectMin().y};
    ImVec2 end_pos = {ImGui::GetItemRectMax().x + STATUS_BORDER_LEFT + STATUS_BORDER_RIGHT + status_size.x,
                      ImGui::GetItemRectMax().y};
    const int max_value = 255;
    ImGui::GetWindowDrawList()->AddRectFilled(
        start_pos, end_pos,
        IM_COL32(info.getStatusColor().red * max_value, info.getStatusColor().green * max_value,
                 info.getStatusColor().blue * max_value, max_value),
        3);

    ImVec4 text_color{static_cast<float>(info.getStatusTextColor().red),
                      static_cast<float>(info.getStatusTextColor().green),
                      static_cast<float>(info.getStatusTextColor().blue), 1};
    ImGui::TextColored(text_color, "%s", status.c_str());

    // render provided infos
    for (const auto& i : info.getParams()) {
        const std::string text = i.first + ": " + i.second;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + LEFT_VALUE_PADDING);
        ImGui::Text("%s", text.c_str());
    }

    ImGui::EndGroup();

    // draw border around
    constexpr float ROUNDING = 4;
    const ImVec2 min_pos = {ImGui::GetItemRectMin().x - MARGIN, ImGui::GetItemRectMin().y - MARGIN};
    const ImVec2 max_pos = {ImGui::GetItemRectMax().x + MARGIN, ImGui::GetItemRectMax().y + MARGIN};
    ImGui::GetWindowDrawList()->AddRect(min_pos, max_pos, IM_COL32(150, 150, 150, 255), ROUNDING,
                                        ImDrawFlags_RoundCornersAll, 1);

    const ImVec2 offset{15, 10};
    const ImVec2 size = max_pos - min_pos + offset;

    if (coord.x == 0) {
        pos.x = size.x;
        if (2 * size.x > ImGui::GetWindowWidth()) {
            // only one row display if windows is too small to see 2 in the same row
            pos.y += size.y;
            pos.x = 0;
        }
    } else {
        pos.x = 0;
        pos.y += size.y;
    }

    return size;
}

}  // namespace luhsoccer::luhviz