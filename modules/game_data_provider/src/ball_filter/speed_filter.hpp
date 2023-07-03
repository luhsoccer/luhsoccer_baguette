#pragma once

#include "base_filter.hpp"
#include "ssl_interface/ssl_types.hpp"
#include "transform/transform.hpp"
#include "logger/logger.hpp"

namespace luhsoccer::game_data_provider {

    class SpeedFilter : public BaseFilter {
    public:
        SpeedFilter();

        bool computeVisionData(const Eigen::Affine2d& latest_ball_position, const Eigen::Affine2d& ball_position,
                               time::TimePoint latest_stamp, time::TimePoint stamp) override;

        static double getBallSpeed(const Eigen::Affine2d& latest_ball_position, const Eigen::Affine2d& ball_position,
                                   time::TimePoint latest_stamp, time::TimePoint stamp);

        [[nodiscard]] bool isActive() const override;

    };
}  // namespace luhsoccer::game_data_provider
