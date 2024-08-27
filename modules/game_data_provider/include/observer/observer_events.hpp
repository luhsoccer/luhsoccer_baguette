#pragma once

#include <utility>
#include "core/common_types.hpp"
#include "core/robot_identifier.hpp"
#include "event_system/event.hpp"
#include "transform/handles.hpp"

namespace luhsoccer::observer {

class DominantTeamChangeEvent : public event_system::Event {
   public:
    DominantTeamChangeEvent(Team dominant_team) : dominant_team(dominant_team) {}

   public:
    Team dominant_team;
};

class ThreatLevelChangedEvent : public event_system::Event {
   public:
    ThreatLevelChangedEvent(RobotIdentifier robot, double old_threat_level, double new_threat_level)
        : robot(robot),                          //
          old_threat_level(old_threat_level),    //
          new_threat_level(new_threat_level) {}  //

   public:
    RobotIdentifier robot;
    double old_threat_level;
    double new_threat_level;
};

class RobotMovedEvent : public event_system::Event {
   public:
    RobotMovedEvent(transform::RobotHandle handle, double distance, Eigen::Vector2d old_position,
                    Eigen::Vector2d new_position)
        : handle(std::move(handle)),
          distance(distance),
          old_position(std::move(old_position)),
          new_position(std::move(new_position)) {}

    transform::RobotHandle handle;
    double distance;
    Eigen::Vector2d old_position;
    Eigen::Vector2d new_position;
};

class BallLeftFieldEvent : public event_system::Event {
   public:
    BallLeftFieldEvent() = default;
};

class BallEnteredAllyPenaltyAreaEvent : public event_system::Event {
   public:
    BallEnteredAllyPenaltyAreaEvent() = default;
};

class BallEnteredEnemyPenaltyAreaEvent : public event_system::Event {
   public:
    BallEnteredEnemyPenaltyAreaEvent() = default;
};

class BallCrossedMiddleLineEvent : public event_system::Event {
   public:
    BallCrossedMiddleLineEvent() = default;
};

class BallCarrierChangedEvent : public event_system::Event {
   public:
    BallCarrierChangedEvent(std::optional<transform::RobotHandle> old_carrier,
                            std::optional<transform::RobotHandle> new_carrier)
        : old_carrier(std::move(old_carrier)), new_carrier(std::move(new_carrier)) {}

    std::optional<transform::RobotHandle> old_carrier;
    std::optional<transform::RobotHandle> new_carrier;
};

class BestInterceptorChangedEvent : public event_system::Event {
   public:
    BestInterceptorChangedEvent(std::optional<transform::RobotHandle> old_interceptor,
                                std::optional<transform::RobotHandle> new_interceptor)
        : old_interceptor(std::move(old_interceptor)), new_interceptor(std::move(new_interceptor)) {}
    std::optional<transform::RobotHandle> old_interceptor;
    std::optional<transform::RobotHandle> new_interceptor;
};

}  // namespace luhsoccer::observer