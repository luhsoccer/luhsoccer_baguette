#pragma once

#include "base_filter.hpp"

namespace luhsoccer::game_data_provider {
/**
 * the moving average filter is a filter that uses the last n values to calculate the average
 * it ranks the latest value exponentially higher than the oldest value
 * the forgetting factor is the factor by which the weight of the oldest value is multiplied
 * the size is the number of values that are used to calculate the average
 */
    class MovingAverageFilter : public BaseFilter {
    public:
        explicit MovingAverageFilter();

        bool computeVisionData(const Eigen::Affine2d& latest_ball_position, const Eigen::Affine2d& ball_position,
                               time::TimePoint latest_stamp, time::TimePoint stamp) override;

        [[nodiscard]] bool isActive() const override;

    private:
        // buffer that stores the ball positions and the time points
        std::vector<std::pair<Eigen::Affine2d, time::TimePoint>> buffer;
        // index of the current position in the buffer
        int current_index = 0;
        // size of the buffer
        int size;

        // adds a new value to the buffer
        void addValue(const Eigen::Affine2d& ball_position, time::TimePoint stamp);
    };
}  // namespace luhsoccer::game_data_provider