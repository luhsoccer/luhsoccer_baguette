#pragma once

#include "robot_identifier.hpp"
#include "ssl_interface/ssl_types.hpp"
#include "transform/transform.hpp"
#include "logger/logger.hpp"

namespace luhsoccer::game_data_provider {
    class BaseFilter {
    public:
        virtual ~BaseFilter() = default;

        explicit BaseFilter();

        /**
         * @brief computes the ball position based on the latest ball position and the time
         *
         * @param latest_ball_position latest ball position
         * @param ball_position current ball position
         * @param latest_stamp latest time point
         * @param stamp current time point
         *
         * @return true if the ball position is valid
         */
        virtual bool computeVisionData(const Eigen::Affine2d& latest_ball_position, const Eigen::Affine2d& ball_position,
                          time::TimePoint latest_stamp, time::TimePoint stamp);

    public:
        [[maybe_unused]] [[nodiscard]] virtual const std::optional<time::TimePoint>& getCurrentStamp() const;

        [[nodiscard]] virtual const std::optional<Eigen::Affine2d>& getCurrentBallPosition() const;

        [[nodiscard]] virtual bool isActive() const;

    protected:
        void setCurrentStamp(const std::optional<time::TimePoint>& currentStamp);

        void setCurrentBallPosition(const std::optional<Eigen::Affine2d>& currentBallPosition);

    private:
        /**
         * @brief latest time point if ball position is known
         *
         */
        std::optional<time::TimePoint> current_stamp;

        /**
         * @brief latest position of the ball if known
         *
         */
        std::optional<Eigen::Affine2d> current_ball_position;
    };
}  // namespace luhsoccer::game_data_provider