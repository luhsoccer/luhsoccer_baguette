

#pragma once

#include "transform/world_model.hpp"

namespace luhsoccer::tests {

void pushFrameToWorldmodel(const std::shared_ptr<luhsoccer::transform::WorldModel>& wm, const std::string& frame_name,
                           time::TimePoint start_time, double start_x, double start_y, double start_theta,
                           double vel_x = 0.0, double vel_y = 0.0, double vel_theta = 0.0, int freq = 100,
                           int num_pushes = 10);

}