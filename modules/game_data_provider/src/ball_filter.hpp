#pragma once

#include "event_system/event_system.hpp"
#include "vision_processor/vision_processor_events.hpp"
#include "robot_interface/events.hpp"
#include "marker_service/marker_service.hpp"
#include "transform/circular_buffer.hpp"

namespace luhsoccer::game_data_provider {

/*
A simple state machine to debounce the light barrier signal
*/
class LightBarrierSM {
   public:
    LightBarrierSM();

    enum class State {
        WAIT_FOR_BALL,
        TRIGGERED_DEBOUNCE,
        TRIGGERED,
        LOST_DEBOUNCE,
    };

    void update();

    bool hasBall();

    State state{State::WAIT_FOR_BALL};
    bool light_barrier{false};
    time::TimePoint last_update{0};
    time::TimePoint next_event{0};
    time::Duration debounce_time;
};

class BallFilter {
   public:
    BallFilter(std::shared_ptr<transform::WorldModel> world_model, marker::MarkerService& marker_service);

    void setup(event_system::EventSystem& system);

   private:
    void onNewProcessedVisionData(const vision_processor::NewProcessedVisionDataEvent& event);

    void onNewRobotFeedback(const robot_interface::RobotFeedbackReceivedEvent& event);

    void publishVisionBall(const Eigen::Affine2d& info);

    bool isLightBarrierPlausible(RobotIdentifier id, time::TimePoint light_barrier_time);

    void publishBallInfo();

    void publishMarker(const transform::BallInfo& ball_info);

    void findEnemiesHoldingBall();

   private:
    std::mutex mutex;
    std::unordered_map<RobotIdentifier, LightBarrierSM> light_barrier_states;
    RobotIdentifier last_ball_holder{RobotIdentifier::create_empty()};
    Eigen::Affine2d last_ball_position;

    std::atomic<time::TimePoint> last_ball_holder_position_time;
    std::optional<Eigen::Affine2d> last_ball_holder_position;
    std::atomic<time::TimePoint> last_ball_holder_time;
    std::atomic<time::TimePoint> last_ball_position_time;
    std::shared_ptr<transform::WorldModel> world_model;
    CircularBuffer<double> last_x_values;
    CircularBuffer<double> last_y_values;

    marker::MarkerService& marker_service;
    marker::LinePlot ball_velocity_plot;

    logger::Logger logger{"BallFilter"};
};

}  // namespace luhsoccer::game_data_provider