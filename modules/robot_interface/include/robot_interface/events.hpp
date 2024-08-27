#pragma once

#include <utility>

#include "event_system/event.hpp"
#include "robot_interface_types.hpp"

namespace luhsoccer::robot_interface {

class RobotCommandSendEvent : public event_system::Event {
   public:
    RobotCommandSendEvent(RobotIdentifier id, RobotCommand command) : id(id), command(std::move(command)) {}

    RobotIdentifier id;
    RobotCommand command;
};

class RobotFeedbackReceivedEvent : public event_system::Event {
   public:
    RobotFeedbackReceivedEvent(RobotIdentifier id, RobotFeedback feedback) : id(id), feedback(std::move(feedback)) {}

    RobotIdentifier id;
    RobotFeedback feedback;
};

};  // namespace luhsoccer::robot_interface