#include "event_system/event_system_helper.hpp"
#include "logger/logger.hpp"

namespace luhsoccer::event_system::helper {

using namespace event_system;

void registerTimerEventDynamic(double frequency, EventSystem& system,
                               const std::function<void(const TimerEvent&)>& callback) {
    static logger::Logger logger("event_system_helper");
    if (frequency <= 0.1) {
        logger.debug("Choosing 10s event for {} Hz", frequency);
        system.registerEventHandler<TimerEvent10Sec>(
            [callback](const EventContext<TimerEvent10Sec> ctx) { callback(ctx.event); });
    } else if (frequency <= 0.2) {
        logger.debug("Choosing 5s event for {} Hz", frequency);
        system.registerEventHandler<TimerEvent5Sec>(
            [callback](const EventContext<TimerEvent5Sec> ctx) { callback(ctx.event); });
    } else if (frequency <= 0.5) {
        logger.debug("Choosing 2s event for {} Hz", frequency);
        system.registerEventHandler<TimerEvent2Sec>(
            [callback](const EventContext<TimerEvent2Sec> ctx) { callback(ctx.event); });
    } else if (frequency <= 1.0) {
        logger.debug("Choosing 1s event for {} Hz", frequency);
        system.registerEventHandler<TimerEvent1Hz>(
            [callback](const EventContext<TimerEvent1Hz> ctx) { callback(ctx.event); });
    } else if (frequency <= 5.0) {
        logger.debug("Choosing 5Hz event for {} Hz", frequency);
        system.registerEventHandler<TimerEvent5Hz>(
            [callback](const EventContext<TimerEvent5Hz> ctx) { callback(ctx.event); });
    } else if (frequency <= 10.0) {
        logger.debug("Choosing 10Hz event for {} Hz", frequency);
        system.registerEventHandler<TimerEvent10Hz>(
            [callback](const EventContext<TimerEvent10Hz> ctx) { callback(ctx.event); });
    } else if (frequency <= 50.0) {
        logger.debug("Choosing 50Hz event for {} Hz", frequency);
        system.registerEventHandler<TimerEvent50Hz>(
            [callback](const EventContext<TimerEvent50Hz> ctx) { callback(ctx.event); });
    } else {
        logger.debug("Choosing 100Hz event for {} Hz", frequency);
        system.registerEventHandler<TimerEvent100Hz>(
            [callback](const EventContext<TimerEvent100Hz> ctx) { callback(ctx.event); });
    }
}

}  // namespace luhsoccer::event_system::helper
