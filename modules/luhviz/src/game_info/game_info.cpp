#include "include/game_info.hpp"

namespace luhsoccer::luhviz {

const static logger::Logger LOGGER{"Game_Info"};

void GameInfo::init() {
    this->robot_icon.create(icon_path, true, true);
    this->unknown_logo.create("res/team_logos/unknown.png", false, true);
}

void GameInfo::render() { this->renderGameInfo(); }

static ImU32 gameStateToColor(luhsoccer::transform::GameState game_state) {
    switch (game_state) {
        case luhsoccer::transform::GameState::HALT:
            return IM_COL32(238, 0, 34, 255);
        case luhsoccer::transform::GameState::STOP:
            return IM_COL32(255, 112, 0, 255);
        case luhsoccer::transform::GameState::NORMAL:
            return IM_COL32(0, 0, 0, 0);

        default:
            return IM_COL32(0, 0, 0, 0);
    }
}

void GameInfo::renderGameInfo() {
    this->display_data.ally_info = this->proxy.getAllyTeamInfo();
    this->display_data.enemy_info = this->proxy.getEnemyTeamInfo();
    this->display_data.game_state = this->proxy.getCurrentGameState();

    ImGui::PushStyleColor(ImGuiCol_Text, this->proxy.accent_text_color);
    ImGui::Begin("Game Info", nullptr, window_flags);
    ImGui::PopStyleColor();

    // draw colored rectangle
    constexpr ImVec2 MAX_RECT_SIZE{150, 100};
    const float y_margin = 50;
    const ImVec2 start_pos{this->window_size.x / 2 - MAX_RECT_SIZE.x / 2, y_margin};
    const ImVec2 end_pos{this->window_size.x / 2 + MAX_RECT_SIZE.x / 2, y_margin + MAX_RECT_SIZE.y};
    const ImVec2 rect_size{end_pos - start_pos};

    if (!this->display_data.ally_info.has_value() && !this->display_data.enemy_info.has_value() &&
        !this->display_data.game_state.has_value()) {
        const std::string text = "No game data present";
        ImVec2 text_size = ImGui::CalcTextSize(text.c_str());
        ImGui::SetCursorPos(start_pos + ImVec2{rect_size.x / 2, rect_size.y * 2 / 5} - text_size / 2);
        ImGui::Text("%s", text.c_str());
        this->window_size = ImGui::GetWindowSize();
        ImGui::End();
        return;
    }

    ImU32 color = IM_COL32(255, 255, 0, 255);  // yellow indicates that no game state information is available
    if (this->display_data.game_state.has_value()) {
        color = gameStateToColor(*this->display_data.game_state);
    }

    ImGui::GetWindowDrawList()->AddRectFilled(ImGui::GetWindowPos() + start_pos, ImGui::GetWindowPos() + end_pos, color,
                                              10);

    // draw first/second half TODO: get half
    ImGui::PushFont(this->fonts.getFont(fonts.FONT_STANDARD));
    const std::string half = "Score";  // first half
    const ImVec2 half_size = ImGui::CalcTextSize(half.c_str());
    ImGui::SetCursorPos(start_pos + ImVec2{rect_size.x / 2, rect_size.y / 7} - half_size / 2);
    ImGui::Text("%s", half.c_str());
    ImGui::PopFont();

    // draw goal score
    ImGui::PushFont(this->fonts.getFont(fonts.FONT_XL));
    size_t score_left = this->display_data.ally_info->score;
    size_t score_right = this->display_data.enemy_info->score;
    const std::string scores = std::to_string(score_left) + " : " + std::to_string(score_right);
    const ImVec2 scores_size = ImGui::CalcTextSize(scores.c_str());
    ImGui::SetCursorPos(start_pos + ImVec2{rect_size.x / 2, rect_size.y * 2 / 5} - scores_size / 2);
    ImGui::Text("%s", scores.c_str());
    ImGui::PopFont();

    // draw game state
    ImGui::PushFont(this->fonts.getFont(fonts.FONT_LARGE));
    const auto gamestate_str = transform::getGameStateName(this->display_data.game_state.value());
    const ImVec2 gamestate_str_size = ImGui::CalcTextSize(gamestate_str.data());
    ImGui::SetCursorPos(start_pos + ImVec2{rect_size.x / 2, rect_size.y * 3 / 4} - gamestate_str_size / 2);
    ImGui::Text("%s", gamestate_str.data());
    ImGui::PopFont();

    // draw team names background
    if (this->display_data.ally_info->name != this->team_name_left || !this->ally_logo.isCreated()) {
        // reload image left team
        this->team_name_left = this->display_data.ally_info->name;
        if (this->display_data.ally_info->name.compare("CMμs") == 0) {
            this->team_name_left = "CMus";
        } else if (this->display_data.ally_info->name.compare("RobôCin") == 0) {
            this->team_name_left = "RoboCin";
        }
        this->ally_logo_known = getTeamLogoTexture(team_name_left, this->ally_logo);
    }
    if (this->display_data.enemy_info->name != this->team_name_right || this->enemy_logo.isCreated()) {
        // reload image right team
        this->team_name_right = this->display_data.enemy_info->name;
        if (this->display_data.enemy_info->name.compare("CMμs") == 0) {
            this->team_name_right = "CMus";
        } else if (this->display_data.enemy_info->name.compare("RobôCin") == 0) {
            this->team_name_right = "RoboCin";
        }
        this->enemy_logo_known = getTeamLogoTexture(team_name_right, this->enemy_logo);
    }

    const ImVec4 color_blue{0, 0, 255, 255};
    const ImVec4 color_yellow{255, 255, 0, 255};

    ImVec4 left_team_color = color_blue;
    ImVec4 right_team_color = color_yellow;
    const bool left_is_blue = config_provider::ConfigProvider::getConfigStore().game_config.is_blue;
    if (left_is_blue) {
        left_team_color = color_blue;
        right_team_color = color_yellow;
    }

    ImGui::PushFont(this->fonts.getFont(fonts.FONT_LARGE));
    const ImVec2 off = {5, 3};
    const ImVec2 left_name_size = ImGui::CalcTextSize(team_name_left.c_str());
    const ImVec2 right_name_size = ImGui::CalcTextSize(team_name_right.c_str());
    const ImVec2 left_name_start_pos = start_pos / ImVec2{2, 1} - ImVec2{left_name_size.x / 2, 0};
    const ImVec2 right_name_start_pos =
        ImVec2{end_pos.x, start_pos.y} + ImVec2{start_pos.x / 2, 0} - ImVec2{right_name_size.x / 2, 0};
    ImGui::GetWindowDrawList()->AddRectFilled(ImGui::GetWindowPos() + left_name_start_pos - off,
                                              ImGui::GetWindowPos() + left_name_start_pos + left_name_size + off,
                                              ImGui::ColorConvertFloat4ToU32(left_team_color), 4);
    ImGui::GetWindowDrawList()->AddRectFilled(ImGui::GetWindowPos() + right_name_start_pos - off,
                                              ImGui::GetWindowPos() + right_name_start_pos + right_name_size + off,
                                              ImGui::ColorConvertFloat4ToU32(right_team_color), 4);
    // draw team names
    ImGui::SetCursorPos(left_name_start_pos);
    if (left_team_color.x == color_blue.x && left_team_color.y == color_blue.y && left_team_color.z == color_blue.z) {
        ImGui::TextColored({1, 1, 1, 1}, "%s", team_name_left.c_str());
    } else {
        ImGui::TextColored({0, 0, 0, 1}, "%s", team_name_left.c_str());
    }

    ImGui::SetCursorPos(right_name_start_pos);
    if (left_team_color.x == color_blue.x && left_team_color.y == color_blue.y && left_team_color.z == color_blue.z) {
        ImGui::TextColored({0, 0, 0, 1}, "%s", team_name_right.c_str());
    } else {
        ImGui::TextColored({1, 1, 1, 1}, "%s", team_name_right.c_str());
    }
    ImGui::PopFont();

    // draw logos
    const ImVec2 logo_size{64, 64};
    const ImVec2 left_logo_pos{start_pos / ImVec2{2, 1} - ImVec2{logo_size.x / 2, -left_name_size.y * 2}};
    const ImVec2 right_logo_pos{ImVec2{end_pos.x, start_pos.y} + ImVec2{start_pos.x / 2, 0} -
                                ImVec2{logo_size.x / 2, -right_name_size.y * 2}};

    // ally logo
    ImGui::SetCursorPos(left_logo_pos);
    if (this->ally_logo_known) {
        ImGui::Image(this->ally_logo.getImguiId(), logo_size);
    } else {
        ImGui::Image(this->unknown_logo.getImguiId(), logo_size);
    }

    // enemy logo
    ImGui::SetCursorPos(right_logo_pos);
    if (this->enemy_logo_known) {
        ImGui::Image(this->enemy_logo.getImguiId(), logo_size);
    } else {
        ImGui::Image(this->unknown_logo.getImguiId(), logo_size);
    }

    // draw yellow/red cards and robot count
    const ImVec2 robot_icon_size{28, 28};
    const ImVec2 card_size{17, 23};
    const float card_x_margin = 7;
    const float card_y_margin = 20;
    const ImVec2 red_card_left_start_pos{left_logo_pos.x, left_logo_pos.y + logo_size.y + card_y_margin};
    const ImVec2 yellow_card_left_start_pos{left_logo_pos.x + card_x_margin + card_size.x,
                                            left_logo_pos.y + logo_size.y + card_y_margin};
    const ImVec2 robot_icon_left_pos{left_logo_pos.x + card_x_margin * 2 + card_size.x * 2,
                                     left_logo_pos.y + logo_size.y + card_y_margin};
    // red card left
    ImGui::GetWindowDrawList()->AddRectFilled(ImGui::GetWindowPos() + red_card_left_start_pos,
                                              ImGui::GetWindowPos() + red_card_left_start_pos + card_size,
                                              IM_COL32(255, 0, 0, 255), 1);
    const size_t red_cards_left = this->display_data.ally_info->red_cards;
    ImGui::SetCursorPos(red_card_left_start_pos + ImVec2{card_size.x / 3 - 1, 1});
    ImGui::PushFont(fonts.getFont(fonts.FONT_LARGE));
    ImGui::Text("%zu", red_cards_left);
    ImGui::PopFont();

    // yellow card left
    ImGui::GetWindowDrawList()->AddRectFilled(ImGui::GetWindowPos() + yellow_card_left_start_pos,
                                              ImGui::GetWindowPos() + yellow_card_left_start_pos + card_size,
                                              IM_COL32(255, 255, 0, 255), 1);
    const size_t yellow_cards_left = this->display_data.ally_info->yellow_cards;
    ImGui::SetCursorPos(yellow_card_left_start_pos + ImVec2{card_size.x / 3 - 1, 1});
    ImGui::PushFont(fonts.getFont(fonts.FONT_LARGE));
    ImGui::TextColored({0, 0, 0, 1}, "%zu", yellow_cards_left);
    ImGui::PopFont();

    // robot count left
    const auto allowed_bots_left = this->display_data.ally_info->max_allowed_bots;
    size_t count_robots_left = 0;
    if (allowed_bots_left.has_value()) {
        count_robots_left = allowed_bots_left.value();
    }
    ImGui::SetCursorPos(robot_icon_left_pos);
    ImGui::Image(this->robot_icon.getImguiId(), robot_icon_size);
    ImGui::SameLine();
    ImGui::PushFont(fonts.getFont(Fonts::FONT_LARGE));
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 3);
    ImGui::Text("%zu", count_robots_left);
    ImGui::PopFont();

    const float x_offset = 15;
    const ImVec2 red_card_right_start_pos{right_logo_pos.x - x_offset, right_logo_pos.y + logo_size.y + card_y_margin};
    const ImVec2 yellow_card_right_start_pos{right_logo_pos.x - x_offset + card_x_margin + card_size.x,
                                             right_logo_pos.y + logo_size.y + card_y_margin};
    const ImVec2 robot_icon_right_pos{right_logo_pos.x - x_offset + card_x_margin * 2 + card_size.x * 2,
                                      right_logo_pos.y + logo_size.y + card_y_margin};
    // red card right
    ImGui::GetWindowDrawList()->AddRectFilled(ImGui::GetWindowPos() + red_card_right_start_pos,
                                              ImGui::GetWindowPos() + red_card_right_start_pos + card_size,
                                              IM_COL32(255, 0, 0, 255), 1);
    const size_t red_cards_right = this->display_data.enemy_info->red_cards;
    ImGui::SetCursorPos(red_card_right_start_pos + ImVec2{card_size.x / 3 - 1, 1});
    ImGui::PushFont(fonts.getFont(fonts.FONT_LARGE));
    ImGui::Text("%zu", red_cards_right);
    ImGui::PopFont();

    // yellow card right
    ImGui::GetWindowDrawList()->AddRectFilled(ImGui::GetWindowPos() + yellow_card_right_start_pos,
                                              ImGui::GetWindowPos() + yellow_card_right_start_pos + card_size,
                                              IM_COL32(255, 255, 0, 255), 1);
    const size_t yellow_cards_right = this->display_data.enemy_info->yellow_cards;
    ImGui::SetCursorPos(yellow_card_right_start_pos + ImVec2{card_size.x / 3 - 1, 1});
    ImGui::PushFont(fonts.getFont(fonts.FONT_LARGE));
    ImGui::TextColored({0, 0, 0, 1}, "%zu", yellow_cards_right);
    ImGui::PopFont();

    // robot count right
    const auto allowed_bots_right = this->display_data.enemy_info->max_allowed_bots;
    size_t count_robots_right = 0;
    if (allowed_bots_right.has_value()) {
        count_robots_right = allowed_bots_right.value();
    }
    ImGui::SetCursorPos(robot_icon_right_pos);
    ImGui::Image(this->robot_icon.getImguiId(), robot_icon_size);
    ImGui::SameLine();
    ImGui::PushFont(fonts.getFont(Fonts::FONT_LARGE));
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 3);
    ImGui::Text("%zu", count_robots_right);
    ImGui::PopFont();

    // draw current game time TODO: get current game time (not available currently)
    // ImGui::PushFont(this->fonts.getFont(fonts.FONT_STANDARD));
    // const std::string game_time = "00:00";
    // const float time_size = 21.5;
    // const float time_y_margin = 20;
    // ImGui::SetCursorPos({this->window_size.x / 2 - time_size, end_pos.y + time_y_margin});
    // ImGui::Text("%s", game_time.c_str());
    // ImGui::PopFont();

    this->window_size = ImGui::GetWindowSize();
    ImGui::End();
}

bool GameInfo::getTeamLogoTexture(const std::string& team_name, GLTexture& logo) {
    std::string team_name_lowe_case = team_name;
    std::transform(team_name_lowe_case.begin(), team_name_lowe_case.end(), team_name_lowe_case.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    const std::string logo_path = "res/team_logos/" + team_name_lowe_case + ".png";
    return logo.create(logo_path, false, true);
}

}  // namespace luhsoccer::luhviz