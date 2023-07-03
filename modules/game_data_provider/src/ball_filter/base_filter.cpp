#include "base_filter.hpp"
#include "ssl_interface/ssl_types.hpp"

namespace luhsoccer::game_data_provider {
    luhsoccer::game_data_provider::BaseFilter::BaseFilter() = default;

    bool BaseFilter::computeVisionData(const Eigen::Affine2d&  /*latest_ball_position*/,
                                       const Eigen::Affine2d&  /*ball_position*/,
                                       const time::TimePoint /*latest_stamp*/, const time::TimePoint /*stamp*/) {
        return false;
    }

    [[maybe_unused]] const std::optional<time::TimePoint>& luhsoccer::game_data_provider::BaseFilter::getCurrentStamp() const {
        return current_stamp;
    }

    const std::optional<Eigen::Affine2d>& luhsoccer::game_data_provider::BaseFilter::getCurrentBallPosition() const {
        return current_ball_position;
    }

    void
    BaseFilter::setCurrentStamp(const std::optional<time::TimePoint>& currentStamp) { current_stamp = currentStamp; }

    void BaseFilter::setCurrentBallPosition(const std::optional<Eigen::Affine2d>& currentBallPosition) {
        current_ball_position = currentBallPosition;
    }

    bool BaseFilter::isActive() const {
        return false;
    }
}  // namespace luhsoccer::game_data_provider