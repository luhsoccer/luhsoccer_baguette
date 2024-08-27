#pragma once

#include "imgui.h"
#include "marker_service/marker_service.hpp"

#include "implot.h"
#include <queue>
#include "marker_service/marker.hpp"
#include "include/data_proxy.hpp"

namespace luhsoccer::luhviz {

class Plotter {
   public:
    Plotter(DataProxy& proxy) : proxy(proxy){};
    void init();
    void render(std::unordered_map<std::string, marker::LinePlot>& line_plots, bool& open);

   private:
    luhsoccer::logger::Logger logger{"luhviz/robot_info_display"};
    DataProxy& proxy;
};
}  // namespace luhsoccer::luhviz
