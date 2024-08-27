#include "include/robert_display.hpp"

namespace luhsoccer::luhviz {

void RobertDisplay::init() {}

void RobertDisplay::render(const std::unordered_map<std::string, marker::RobotInfo>& robot_infos, bool& open) {
    if (!open) {
        return;
    }

    ImGui::PushStyleColor(ImGuiCol_Text, this->proxy.accent_text_color);
    ImGui::Begin("Robert Display", &open, window_flags);
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
    Eigen::Vector2i coord{0, 0};
    ImVec2 block_position{LEFT_PADDING, TOP_PADDING};
    int num_columns = 1;
    float max_height_in_column = 0;
    for (const auto& r_info : this->sorted_robot_infos) {
        ImVec2 block_size = renderRobotInfoBlock(r_info, block_position, ImGui::GetWindowWidth());
        num_columns = static_cast<int>((ImGui::GetWindowWidth() - 2 * LEFT_PADDING) / (block_size.x - 2 * MARGIN));
        max_height_in_column = std::max(max_height_in_column, block_size.y);
        if (coord.x() == num_columns - 1) {
            coord.x() = 0;
            coord.y() += 1;
            block_position = {LEFT_PADDING, block_position.y + max_height_in_column};
            max_height_in_column = 0;
        } else {
            coord.x() += 1;
            block_position = {block_position.x + block_size.x, block_position.y};
        }
    }

    ImGui::End();
}

std::vector<std::string> wrapText(const std::string& text, double size) {
    std::vector<std::string> all_lines;
    std::istringstream iss_original(text);
    std::string line_text;
    while (std::getline(iss_original, line_text, '\n')) {
        std::vector<std::string> words;
        std::istringstream iss(line_text);
        std::string word;
        while (iss >> word) {
            words.push_back(word);
        }
        std::vector<std::string> lines;
        std::string current_line;
        double current_line_width = 0;
        double max_line_width = size;

        for (const std::string& word : words) {
            double word_width = ImGui::CalcTextSize(word.c_str()).x;

            if (current_line_width + word_width <= max_line_width) {
                // Add word to current line
                if (!current_line.empty()) {
                    current_line += " ";
                    current_line_width += ImGui::CalcTextSize(" ").x;
                }
                current_line += word;
                current_line_width += word_width;
            } else {
                // Start a new line
                lines.push_back(current_line);
                current_line = word;
                current_line_width = word_width;
            }
        }

        // Add the last line
        if (!current_line.empty()) {
            lines.push_back(current_line);
        }
        all_lines.insert(all_lines.end(), lines.begin(), lines.end());
    }

    return all_lines;
}

ImVec2 RobertDisplay::renderRobotInfoBlock(const marker::RobotInfo& info, ImVec2& pos, float window_width) {
    ImGui::SetCursorPos(pos);
    ImGui::BeginGroup();
    // render Robot id
    ImGui::Text("Robot %zu", info.getRobotId().id);

    constexpr int MIN_BOX_WIDTH = 280;
    int num_columns = static_cast<int>(window_width / MIN_BOX_WIDTH);
    const float box_width =
        (window_width - 2 * LEFT_PADDING - 2 * MARGIN) / static_cast<float>(num_columns) - 2 * MARGIN;
    constexpr int BOX_HEIGHT = 400;
    // logger.info("Window width: {:0.2f} box width: {:0.2f} num_columns: {}", window_width, box_width, num_columns);

    // render badges
    for (const auto& [key, badge] : info.getBadges()) {
        if (badge.message.empty()) continue;
        ImVec2 status_size = ImGui::CalcTextSize(badge.message.c_str());
        ImGui::SameLine();
        if (ImGui::GetCursorPosX() - pos.x + status_size.x > box_width) {
            ImGui::NewLine();
        }

        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + STATUS_PADDING_LEFT);
        ImVec2 start_pos = {ImGui::GetCursorScreenPos().x - 3.0f, ImGui::GetCursorScreenPos().y};
        ImVec2 end_pos = {ImGui::GetCursorScreenPos().x + 3.0f + status_size.x,
                          ImGui::GetCursorScreenPos().y + status_size.y + 1};
        const int max_value = 255;
        ImGui::GetWindowDrawList()->AddRectFilled(start_pos, end_pos,
                                                  IM_COL32(badge.color.red * max_value, badge.color.green * max_value,
                                                           badge.color.blue * max_value, max_value),
                                                  3);

        ImVec4 text_color{static_cast<float>(badge.text_color.red), static_cast<float>(badge.text_color.green),
                          static_cast<float>(badge.text_color.blue), 1};
        ImGui::TextColored(text_color, "%s", badge.message.c_str());
    }
    // render provided infos
    for (const auto& i : info.getParams()) {
        const std::string text = i.first + ": " + i.second;
        bool first = true;
        auto lines = wrapText(text.c_str(), box_width - LEFT_VALUE_PADDING);
        for (const auto& line : lines) {
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + LEFT_VALUE_PADDING);
            if (first) {
                ImGui::Text("%s", line.c_str());
                first = false;
            } else {
                ImGui::Text("  %s", line.c_str());
            }
        }
    }

    ImGui::EndGroup();

    // draw border around
    constexpr float ROUNDING = 4;

    const ImVec2 min_pos = {ImGui::GetItemRectMin().x - MARGIN, ImGui::GetItemRectMin().y - MARGIN};
    const ImVec2 max_pos = {ImGui::GetItemRectMin().x + box_width + MARGIN, ImGui::GetItemRectMax().y + MARGIN};
    ImGui::GetWindowDrawList()->AddRect(min_pos, max_pos, IM_COL32(150, 150, 150, 255), ROUNDING,
                                        ImDrawFlags_RoundCornersAll, 1);

    const ImVec2 offset{LEFT_PADDING - MARGIN, 5};
    const ImVec2 size = max_pos - min_pos + offset;

    return size;
}

}  // namespace luhsoccer::luhviz