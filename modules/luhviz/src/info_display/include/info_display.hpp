#pragma once

#include "imgui.h"
#include "marker_service/marker_service.hpp"
#include "common/include/fonts.hpp"
#include "include/data_proxy.hpp"

namespace luhsoccer::luhviz {

class InfoDisplay {
   public:
    InfoDisplay(DataProxy& proxy) : proxy(proxy) {}
    void init();
    void render(std::map<std::string, std::map<std::string, marker::Info>>& infos, bool& open);

   private:
    luhsoccer::logger::Logger logger{"luhviz/info_display"};
    DataProxy& proxy;

    constexpr static float MARGIN = 8.0f;
    constexpr static float STATUS_PADDING_LEFT = 3.0f;
    constexpr static float LEFT_PADDING = MARGIN + 5.0f;
    constexpr static float LEFT_VALUE_PADDING = MARGIN + 15.0f;
    constexpr static float TOP_PADDING = MARGIN + 3.0f;
    constexpr static float STATUS_BORDER_LEFT = 8.0f;
    constexpr static float STATUS_BORDER_RIGHT = 5.0f;
};
}  // namespace luhsoccer::luhviz