#pragma once

#include "event_system/event.hpp"
#include <transform/game_state.hpp>
#include <transform/handles.hpp>
#include <utility>

namespace luhsoccer::ssl_interface {
enum class SSLStage : int;
}

namespace luhsoccer::transform {
class WorldModel;
}
namespace luhsoccer::game_data_provider {

class BallShotEvent : public event_system::Event {
   public:
    BallShotEvent(Eigen::Vector2d old_vel, Eigen::Vector2d new_vel)
        : old_velocity(std::move(old_vel)), new_velocity(std::move(new_vel)) {}

   public:
    Eigen::Vector2d old_velocity;
    Eigen::Vector2d new_velocity;
};

class GameStateChangedEvent : public event_system::Event {
   public:
    GameStateChangedEvent(transform::GameState old_state, transform::GameState new_state)
        : old_state{old_state}, new_state{new_state} {}

    transform::GameState old_state;
    transform::GameState new_state;
};

class RealWorldModelUpdatedEvent : public event_system::Event {
   public:
    RealWorldModelUpdatedEvent(std::shared_ptr<const transform::WorldModel> world_model,
                               time::TimePoint timestamp_capture, time::TimePoint timestamp_sent)
        : world_model{std::move(world_model)}, timestamp_capture(timestamp_capture), timestamp_sent(timestamp_sent) {}

    std::shared_ptr<const transform::WorldModel> world_model;
    time::TimePoint timestamp_capture;
    time::TimePoint timestamp_sent;
};

class FieldDataUpdatedEvent : public event_system::Event {
   public:
    FieldDataUpdatedEvent(transform::FieldData field_data) : field_data(field_data) {}

    transform::FieldData field_data;
};

class RobotRemovedFromFieldEvent : public event_system::Event {
   public:
    RobotRemovedFromFieldEvent(transform::RobotHandle robot) : robot{std::move(robot)} {}

    transform::RobotHandle robot;
};

class GameStageChangedEvent : public event_system::Event {
   public:
    GameStageChangedEvent(ssl_interface::SSLStage new_stage) : stage(new_stage) {}

    luhsoccer::ssl_interface::SSLStage stage;
};

}  // namespace luhsoccer::game_data_provider
