#pragma once

#include "marker.hpp"
#include "marker_impl.hpp"
#include "marker_2d_impl.hpp"

namespace luhsoccer::marker {

struct LuhvizMarkers {
    std::unordered_map<std::string, MarkerImpl> markers;
    std::unordered_map<std::string, Marker2DImpl> markers2d;
    std::map<std::string, std::map<std::string, Info>> info_markers;
    std::unordered_map<std::string, RobotInfo> robot_info_markers;
    std::unordered_map<std::string, LinePlot> plots;
};

};  // namespace luhsoccer::marker