
#include "world_model_test_helper.hpp"

namespace luhsoccer::tests {
void pushFrameToWorldmodel(const std::shared_ptr<luhsoccer::transform::WorldModel> &wm, const std::string &frame_name,
                           time::TimePoint start_time, double start_x, double start_y, double start_theta, double vel_x,
                           double vel_y, double vel_theta, int freq, int num_pushes) {
    for (int i = 0; i < num_pushes; i++) {
        transform::TransformWithVelocity t;
        t.header.child_frame = frame_name;
        t.header.parent_frame = wm->getGlobalFrame();
        t.header.stamp = start_time + i * time::Duration(1.0 / freq);
        t.transform = Eigen::Translation2d(start_x + i * vel_x / freq, start_y + i * vel_y / freq) *
                      Eigen::Rotation2Dd(start_theta + vel_theta * i / freq);
        wm->pushTransform(t, false);
    }
}
}  // namespace luhsoccer::tests