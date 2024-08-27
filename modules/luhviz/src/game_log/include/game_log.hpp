#pragma once

#include <cstdio>
#include <cmath>
#include "imgui.h"
#include "game_data_provider/game_data_provider.hpp"
#include "inspector/include/inspector.hpp"
#include "marker_service/marker_service.hpp"
#include "new_rendering/include/gl_texture.hpp"

namespace luhsoccer::luhviz {

class GameLog {
   public:
    GameLog(DataProxy& proxy, Fonts& fonts) : proxy(proxy), fonts(fonts){};
    void init();
    void render(bool& open);

   private:
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;

    logger::Logger logger{"luhviz/game_log"};
    DataProxy& proxy;
    Fonts& fonts;

    int last_replay_factor{1};
    int replay_factor{1};

    int preload_paths{0};
    int last_preload_paths{0};

    bool x10{false};

    ImVec2 window_size{0, 0};
};
}  // namespace luhsoccer::luhviz