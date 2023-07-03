#pragma once

#include <utility>
#include "robot_identifier.hpp"
#include "robot_interface/robot_interface_types.hpp"
#include "transform/transform.hpp"
#include "logger/logger.hpp"

namespace luhsoccer::robot_interface {
struct RobotFeedback;
}

namespace luhsoccer::game_data_provider {

template <class T>
using InfoMember = std::optional<std::pair<T, time::TimePoint>>;

struct RobotInfo {
    InfoMember<Eigen::Affine2d> transform;
    InfoMember<Eigen::Vector3d> velocity;
    InfoMember<bool> on_field;
    InfoMember<bool> has_ball;
    InfoMember<int> cap_voltage;
};
// NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
using Vector6d = Eigen::Matrix<double, 6, 1>;
// NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
using Matrix6d = Eigen::Matrix<double, 6, 6>;

enum class RobotFilterMode { OUT_OF_FIELD, VISION_WITH_FEEDBACK, VISION_WITH_COMMAND, VISION, ROBOT_FEEDBACK };

std::ostream& operator<<(std::ostream& out, const RobotFilterMode value);

class KalmanFilter {
   public:
    KalmanFilter(double step_time = 1.0 / 100.0);

    std::pair<Vector6d, Matrix6d> predictStepCommand(Vector6d robot_state, Matrix6d kalman_p,
                                                     const Eigen::Vector3d& velocity, double step_time);
    std::pair<Vector6d, Matrix6d> predictStepFeedback(Vector6d robot_state, Matrix6d kalman_p,
                                                      const Eigen::Vector3d& velocity, double step_time);
    std::pair<Vector6d, Matrix6d> correctionStepVision(const Vector6d& robot_state, const Matrix6d& kalman_p,
                                                       const Vector6d& measurement);
    void initMatrices(double step_time, bool force_update = false);

   private:
    Matrix6d kalman_a;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
    Eigen::Matrix<double, 6, 3> kalman_b;
    Matrix6d kalman_c;

    Matrix6d kalman_q;
    Matrix6d kalman_r;
    double last_step_time{0.0};
};

constexpr double DEFAULT_FILTER_FREQUENCY = 100;
class RobotDataFilter {
   public:
    RobotDataFilter(const RobotIdentifier& id, std::string global_frame,
                    time::Duration invalid_interval = time::Duration(1.0), double rate = DEFAULT_FILTER_FREQUENCY);
    // void setRobotOnField(bool on_field, time::TimePoint time = time::TimePoint(0));
    bool addVisionData(const Eigen::Affine2d& transform, double process_delay_ms,
                       time::TimePoint time = time::TimePoint(0));
    bool addFeedbackData(const robot_interface::RobotFeedback& feedback);
    bool addCommandSend(const robot_interface::RobotCommand& command);

    bool setBallInDribbler(bool ball_in_dribbler, time::TimePoint time = time::TimePoint(0));

    RobotFilterMode getFilterMode() const {
        const std::lock_guard<std::mutex> lock(this->mtx);
        return this->mode;
    }
    transform::TransformWithVelocity getLatestTransform();
    transform::AllyRobotData getLatestRobotData();

    void updateParams() {
        const std::lock_guard<std::mutex> lock(this->mtx);
        this->kalman_filter.initMatrices(1.0 / this->rate, true);
    }

   private:
    void updateMode(time::TimePoint time = time::TimePoint(0));

    double rate;
    KalmanFilter kalman_filter;
    std::optional<Vector6d> kalman_state;
    std::optional<Matrix6d> kalman_p;

    std::deque<std::pair<time::TimePoint, Eigen::Vector3d>> velocity_command_queue;
    std::deque<std::pair<time::TimePoint, Eigen::Vector3d>> velocity_feedback_queue;

    RobotIdentifier id;
    RobotInfo latest_info;
    std::optional<std::pair<time::TimePoint, bool>>
        last_feedback_time_and_type;  // second true if with position feedback
    std::optional<time::TimePoint> last_vision_time;
    std::optional<Eigen::Vector3d> last_vision_position;
    std::optional<time::TimePoint> last_command_time;
    RobotFilterMode mode;
    time::Duration invalid_interval;
    mutable std::mutex mtx;
    std::string global_frame;
    logger::Logger logger{"RobotDataFilter"};
    time::LoopStopwatch watch{"RobotDataFilterFeedback"};
};
}  // namespace luhsoccer::game_data_provider