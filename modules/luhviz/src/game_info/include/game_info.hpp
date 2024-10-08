#pragma once

#include <cstdio>
#include <cmath>
#include "imgui.h"
#include "game_data_provider/game_data_provider.hpp"
#include "inspector/include/inspector.hpp"
#include "marker_service/marker_service.hpp"
#include "new_rendering/include/gl_texture.hpp"

namespace luhsoccer::luhviz {

class GameInfo {
   public:
    GameInfo(DataProxy& proxy, Fonts& fonts) : proxy(proxy), fonts(fonts){};
    void init();
    void render(bool& open);

   private:
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;

    const std::string icon_path = "res/images/robot_icon.png";
    std::string team_name_left = "res/team_logos/luhbots.png";
    std::string team_name_right = "res/team_logos/unknown_logo.png";

    logger::Logger logger{"luhviz/game_info"};
    DataProxy& proxy;
    GameDataDisplay display_data{};
    GLTexture robot_icon{};
    Fonts& fonts;

    std::string current_gamestate;
    std::string last_gamestate;
    std::string second_last_gamestate;

    GLTexture ally_logo{};
    GLTexture enemy_logo{};
    GLTexture unknown_logo{};
    bool ally_logo_known{false};
    bool enemy_logo_known{false};

    ImVec2 window_size{0, 0};

    void renderGameInfo(bool& open);
    void renderStateDisplays();
    bool getTeamLogoTexture(const std::string& team_name, GLTexture& logo);
};
}  // namespace luhsoccer::luhviz