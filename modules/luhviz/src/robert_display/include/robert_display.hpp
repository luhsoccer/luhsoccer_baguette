#pragma once

#include "imgui.h"
#include "marker_service/marker_service.hpp"
#include "include/data_proxy.hpp"

namespace luhsoccer::luhviz {

class RobertDisplay {
   public:
    RobertDisplay(DataProxy& proxy) : proxy(proxy) {}
    void init();
    void render(const std::unordered_map<std::string, marker::RobotInfo>& robot_infos, bool& open);

   private:
    luhsoccer::logger::Logger logger{"luhviz/robot_info_display"};

    constexpr static float MARGIN = 8.0f;
    constexpr static float STATUS_PADDING_LEFT = 3.0f;
    constexpr static float LEFT_PADDING = MARGIN + 5.0f;
    constexpr static float LEFT_VALUE_PADDING = MARGIN + 3.0f;
    constexpr static float TOP_PADDING = MARGIN + 30.0f;
    constexpr static float STATUS_BORDER_LEFT = 8.0f;
    constexpr static float STATUS_BORDER_RIGHT = 5.0f;

    const ImGuiWindowFlags window_flags = ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoScrollWithMouse;

    std::vector<marker::RobotInfo> sorted_robot_infos{};
    ImVec2 window_size{50, 50};
    DataProxy& proxy;

    /**
     * @brief renders a block of one robot with its given infos
     *
     * @param title
     * @param info
     */
    ImVec2 renderRobotInfoBlock(const marker::RobotInfo& info, ImVec2& pos, float window_width);
};
}  // namespace luhsoccer::luhviz