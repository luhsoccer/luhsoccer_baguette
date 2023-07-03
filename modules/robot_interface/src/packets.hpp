#pragma once

#include <robot_interface/robot_interface_types.hpp>

namespace luhsoccer::robot_interface {

struct RobotCommandWrapper {
    uint32_t id{0};
    TeamColor color{TeamColor::BLUE};
    time::TimePoint time_point{0.0};
    RobotCommand cmd{};
};

struct RobotFeedbackWrapper {
    uint32_t id{0};
    TeamColor color{TeamColor::BLUE};
    RobotFeedback feedback{};
};

class PacketBuilder {
   public:
    PacketBuilder() = default;
    PacketBuilder(const PacketBuilder &) = default;
    PacketBuilder(PacketBuilder &&) = delete;
    PacketBuilder &operator=(const PacketBuilder &) = default;
    PacketBuilder &operator=(PacketBuilder &&) = delete;
    virtual ~PacketBuilder() = default;

    virtual void addMessage(const RobotCommandWrapper &cmd) = 0;
    virtual std::vector<RobotFeedbackWrapper> buildAndSend() = 0;
};

}  // namespace luhsoccer::robot_interface