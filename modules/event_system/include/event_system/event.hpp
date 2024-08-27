#pragma once

#include <time/time.hpp>

namespace luhsoccer::event_system {

class Event {
   public:
    Event() = default;
    Event(const Event&) noexcept = default;
    Event(Event&&) noexcept = default;
    Event& operator=(const Event&) noexcept = default;
    Event& operator=(Event&&) noexcept = default;
    virtual ~Event() = default;

    time::TimePoint created_at{time::now()};
};

}  // namespace luhsoccer::event_system