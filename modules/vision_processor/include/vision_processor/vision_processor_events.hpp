#pragma once

#include "core/common_types.hpp"
#include "event_system/event.hpp"
#include <variant>

namespace luhsoccer::ssl_interface {
struct SSLVisionData;
}  // namespace luhsoccer::ssl_interface

namespace luhsoccer::vision_processor {

struct ProcessedVisionData {
    size_t camera_id;
    time::TimePoint time_captured;
    time::TimePoint time_sent;

    std::optional<Eigen::Affine2d> ball;

    std::array<std::optional<Eigen::Affine2d>, MAX_ROBOTS_PER_TEAM> blue_robots;
    std::array<std::optional<Eigen::Affine2d>, MAX_ROBOTS_PER_TEAM> yellow_robots;
};

class NewProcessedVisionDataEvent : public event_system::Event {
   public:
    NewProcessedVisionDataEvent(ProcessedVisionData data) : data(std::move(data)) {}
    ProcessedVisionData data;
};

struct TeleportData {
    struct BallTeleport {};
    struct RobotTeleport {
        RobotTeleport(size_t id, TeamColor team) : id(id), team(team){};
        size_t id;
        TeamColor team;
    };

    std::variant<BallTeleport, RobotTeleport> teleport_type{BallTeleport()};

    Eigen::Affine2d new_pose{Eigen::Affine2d::Identity()};
    Eigen::Vector3d new_velocity{0.0, 0.0, 0.0};

    time::TimePoint time{time::TimePoint(0)};
};

class TeleportEvent : public event_system::Event {
   public:
    TeleportEvent(TeleportData data) : data(std::move(data)) {}
    TeleportData data;
};

}  // namespace luhsoccer::vision_processor
