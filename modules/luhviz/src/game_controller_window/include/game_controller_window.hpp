#pragma once

#include "imgui.h"
#define IMGUI_DEFINE_MATH_OPERATORS
#include "imgui_internal.h"
#include "logger/logger.hpp"
#include "include/data_proxy.hpp"
#include "game_controller_interface/game_controller_interface.hpp"
#include "imgui_backend/imgui_stdlib.h"

namespace luhsoccer::luhviz {

class GameControllerWindow {
   public:
    GameControllerWindow(DataProxy& proxy) : proxy(proxy) {}
    void init();
    void render(bool* open);

   private:
    luhsoccer::logger::Logger logger{"luhviz/game_controller_window"};

    const std::string start_str = "Start GC";
    const std::string stop_str = "Stop GC";
    const float btn_offset{10};
    const ImVec2 start_btn_offset{510, 80};
    DataProxy& proxy;
    std::string gc_path{"empty"};
    std::unique_ptr<game_controller_interface::GameControllerInterface> gc_interface;
};

}  // namespace luhsoccer::luhviz