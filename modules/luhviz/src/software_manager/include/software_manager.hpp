#pragma once

#include <cstdio>
#include <cmath>
#include "imgui.h"
#include "new_rendering/include/gl_texture.hpp"
#include "include/data_proxy.hpp"
#include "common/include/fonts.hpp"

namespace luhsoccer::luhviz {

class SoftwareManager {
   public:
    SoftwareManager(DataProxy& proxy, Fonts& fonts) : proxy(proxy), fonts(fonts){};
    void init();
    void render(bool& open);

   private:
    const ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;

    logger::Logger logger{"luhviz/software_manager"};
    DataProxy& proxy;
    Fonts& fonts;

    ImVec2 window_size{0, 0};
};
}  // namespace luhsoccer::luhviz