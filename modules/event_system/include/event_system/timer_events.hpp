#pragma once

#include "event.hpp"

namespace luhsoccer::event_system {
class TimerEvent : public Event {
   public:
    TimerEvent(std::chrono::milliseconds period) : period(period) {}

    [[nodiscard]] std::chrono::milliseconds getPeriod() const { return period; }

   private:
    std::chrono::milliseconds period;
};

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers)
class TimerEvent100Hz : public TimerEvent {
   public:
    TimerEvent100Hz() : TimerEvent(std::chrono::milliseconds(10)) {}
};

class TimerEvent50Hz : public TimerEvent {
   public:
    TimerEvent50Hz() : TimerEvent(std::chrono::milliseconds(20)) {}
};

class TimerEvent10Hz : public TimerEvent {
   public:
    TimerEvent10Hz() : TimerEvent(std::chrono::milliseconds(100)) {}
};

class TimerEvent5Hz : public TimerEvent {
   public:
    TimerEvent5Hz() : TimerEvent(std::chrono::milliseconds(200)) {}
};

class TimerEvent1Hz : public TimerEvent {
   public:
    TimerEvent1Hz() : TimerEvent(std::chrono::milliseconds(1000)) {}
};

class TimerEvent2Sec : public TimerEvent {
   public:
    TimerEvent2Sec() : TimerEvent(std::chrono::milliseconds(2000)) {}
};

class TimerEvent5Sec : public TimerEvent {
   public:
    TimerEvent5Sec() : TimerEvent(std::chrono::milliseconds(5000)) {}
};

class TimerEvent10Sec : public TimerEvent {
   public:
    TimerEvent10Sec() : TimerEvent(std::chrono::milliseconds(10000)) {}
};
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers)
}  // namespace luhsoccer::event_system