#pragma once

#include <event_system/event.hpp>

namespace luhsoccer {

class StartEvent : public event_system::Event {
   public:
    StartEvent() = default;
};

class StopEvent : public event_system::Event {
   public:
    StopEvent() = default;
};

}  // namespace luhsoccer