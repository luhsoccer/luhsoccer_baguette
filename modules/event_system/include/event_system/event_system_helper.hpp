#pragma once

#include "timer_events.hpp"
#include "event_system.hpp"

namespace luhsoccer::event_system::helper {

void registerTimerEventDynamic(double frequency, event_system::EventSystem& system,
                               const std::function<void(const TimerEvent&)>&);
}  // namespace luhsoccer::event_system::helper