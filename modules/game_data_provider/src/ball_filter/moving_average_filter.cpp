#include "moving_average_filter.hpp"
#include "config_provider/config_store_main.hpp"

namespace luhsoccer::game_data_provider {
MovingAverageFilter::MovingAverageFilter() {
    const auto& cs = luhsoccer::config_provider::ConfigProvider::getConfigStore();
    size = cs.ball_filter_config.moving_average_filter_size;
    buffer = std::vector<std::pair<Eigen::Affine2d, time::TimePoint>>(size);
}

bool MovingAverageFilter::computeVisionData(const Eigen::Affine2d& /*latest_ball_position*/,
                                            const Eigen::Affine2d& ball_position, time::TimePoint /*latest_stamp*/,
                                            time::TimePoint stamp) {
    const auto& cs = luhsoccer::config_provider::ConfigProvider::getConfigStore();
    addValue(ball_position, stamp);
    Eigen::Affine2d resulting_ball_position = Eigen::Affine2d::Identity();
    double weight = 1;
    double sum = 0;
    for (int i = 0; i < size; i++) {
        resulting_ball_position.translation() += weight * buffer[i].first.translation();
        sum += weight;
        weight *= cs.ball_filter_config.moving_average_filter_forgetting_factor * 2;
    }
    resulting_ball_position.translation() /= sum;

    setCurrentBallPosition(resulting_ball_position);
    setCurrentStamp(stamp);
    return true;
}

void MovingAverageFilter::addValue(const Eigen::Affine2d& ball_position, time::TimePoint stamp) {
    buffer[current_index] = std::make_pair(ball_position, stamp);
    current_index = (current_index + 1) % size;
}

bool MovingAverageFilter::isActive() const {
    const auto& cs = luhsoccer::config_provider::ConfigProvider::getConfigStore();
    return cs.ball_filter_config.use_moving_average_filter;
}

}  // namespace luhsoccer::game_data_provider
