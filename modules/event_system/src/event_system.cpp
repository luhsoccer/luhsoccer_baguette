#include "event_system/event_system.hpp"
#include "event_system/timer_events.hpp"

#include "utils/utils.hpp"
#include <asio.hpp>

namespace luhsoccer::event_system {

constexpr std::size_t IO_THREADS = 2;

EventSystem::EventSystem()
    : io_context(std::make_unique<asio::io_context>()), thread_pool(std::thread::hardware_concurrency() + IO_THREADS) {}

EventSystem::~EventSystem() = default;

asio::io_context& EventSystem::getIoContext() { return *this->io_context; }

void EventSystem::startIOEvents() {
    for (std::size_t i = 0; i < IO_THREADS; i++) {
        this->thread_pool.detach_task([this]() { this->io_context->run(); });
    }
}

void EventSystem::stopIOEvents() { this->io_context->stop(); }

void EventSystem::startTimerEvents() {
    this->timer_events_running = true;
    this->thread_pool.detach_task([this]() {
        uint64_t counter = 0;
        while (this->timer_events_running) {
            highPrecisionSleep(std::chrono::milliseconds(10));
            counter++;
            if (counter % 1 == 0) {
                this->fireEvent(TimerEvent100Hz{});
            }
            if (counter % 2 == 0) {
                this->fireEvent(TimerEvent50Hz{});
            }
            if (counter % 10 == 0) {
                this->fireEvent(TimerEvent10Hz{});
            }
            if (counter % 20 == 0) {
                this->fireEvent(TimerEvent5Hz{});
            }
            if (counter % 100 == 0) {
                this->fireEvent(TimerEvent1Hz{});
            }
            if (counter % 200 == 0) {
                this->fireEvent(TimerEvent2Sec{});
            }
            if (counter % 500 == 0) {
                this->fireEvent(TimerEvent5Sec{});
            }
            if (counter % 1000 == 0) {
                this->fireEvent(TimerEvent10Sec{});
            }
        }
    });
}

void EventSystem::registerPolymorphicEventHandler(const std::type_info& type,
                                                  std::function<void(const std::shared_ptr<Event>&)> event_handler,
                                                  bool allow_parallel_execution, std::source_location location) {
    if (!freeze_registration.test()) {
        std::unique_lock lock(this->mutex);
        this->handler[std::type_index(type)].emplace_back(std::move(event_handler), allow_parallel_execution, location);
    } else {
        throw std::runtime_error("Event registration is only allowed at setup time");
    }
}

void EventSystem::fireEvent(const std::shared_ptr<Event>& event) {
    this->fireEvent(std::type_index(typeid(*event)), event);
}

void EventSystem::fireEvent(const std::type_index& type, const std::shared_ptr<Event>& event) {
    if (freeze_registration.test()) {
        std::shared_lock lock(this->mutex);
        auto it = this->handler.find(type);
        if (it != this->handler.end()) {
            for (auto& handler : it->second) {
                if (!handler.allow_parallel_execution && handler.running.test_and_set()) {
                    handler.missed_event_counter++;

                    constexpr int MISSED_EVENT_THRESHOLD = 99;
                    if (handler.missed_event_counter.load() > MISSED_EVENT_THRESHOLD) {
                        logger.warning("Event handler {}:{} is running behind. {} events have been missed",
                                       handler.location.file_name(), handler.location.line(),
                                       handler.missed_event_counter.load());
                        handler.missed_event_counter.exchange(0);
                    }

                    continue;
                }

                this->thread_pool.detach_task([&handler, event_ptr = event]() {
                    handler.callback(event_ptr);
                    handler.running.clear();
                });
            }
        }
    }
}

void EventSystem::lockEventRegistration() {
    this->freeze_registration.test_and_set();
    logger.info("Freezing event registration");
    std::size_t event_count = 0;
    std::size_t handler_count = 0;
    logger.debug("Event system stats:");
    for (auto& [type, handlers] : this->handler) {
        event_count++;
        handler_count += handlers.size();
        logger.debug("Registers {} event handlers for event {}", handlers.size(), type.name());
    }
    logger.debug("Total: Registered {} events with {} handlers", event_count, handler_count);
}

void EventSystem::stopTimerEvents() {
    logger.debug("Stopping timer events");
    this->timer_events_running = false;
    // this->thread_pool.pause();
    // std::unique_lock lock(this->mutex);
    // this->handler.clear();
}
};  // namespace luhsoccer::event_system