#include "speed_filter.hpp"
#include "config_provider/config_store_main.hpp"

namespace luhsoccer::game_data_provider {

SpeedFilter::SpeedFilter() = default;

double SpeedFilter::getBallSpeed(const Eigen::Affine2d& latest_ball_position, const Eigen::Affine2d& ball_position,
                                 const time::TimePoint latest_stamp, const time::TimePoint stamp) {
    if (stamp.asSec() - latest_stamp.asSec() == 0) {
        return 0;
    }
    return (ball_position.translation() - latest_ball_position.translation()).norm() /
           (stamp.asSec() - latest_stamp.asSec());
}

bool SpeedFilter::computeVisionData(const Eigen::Affine2d& latest_ball_position, const Eigen::Affine2d& ball_position,
                                    time::TimePoint latest_stamp, time::TimePoint stamp) {
    const auto& cs = luhsoccer::config_provider::ConfigProvider::getConfigStore();
    bool const ball_position_valid = getBallSpeed(latest_ball_position, ball_position, latest_stamp, stamp) <=
                                     cs.ball_filter_config.speed_filter_max_speed;
    if (ball_position_valid) {
        setCurrentBallPosition(ball_position);
        setCurrentStamp(stamp);
    }
    return ball_position_valid;
}

bool SpeedFilter::isActive() const {
    const auto& cs = luhsoccer::config_provider::ConfigProvider::getConfigStore();
    return cs.ball_filter_config.use_speed_filter;
}
}  // namespace luhsoccer::game_data_provider