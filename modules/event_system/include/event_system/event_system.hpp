#pragma once

#include <unordered_map>
#include <functional>
#include <utility>
#include <vector>
#include <shared_mutex>
#include <typeindex>
#include <atomic>

#include "event.hpp"

#include "BS_thread_pool.hpp"
#include "logger/logger.hpp"

namespace asio {
class io_context;
}

namespace luhsoccer::event_system {

class EventHandler {
   public:
    EventHandler(std::function<void(const std::shared_ptr<Event>&)> callback, bool allow_parallel_execution,
                 std::source_location location)
        : allow_parallel_execution(allow_parallel_execution), location(location), callback(std::move(callback)) {}
    EventHandler(const EventHandler& other) = delete;
    EventHandler& operator=(const EventHandler& other) = delete;
    EventHandler(EventHandler&& other) noexcept
        : allow_parallel_execution(other.allow_parallel_execution),
          missed_event_counter(other.missed_event_counter.load()),
          callback(std::move(other.callback)) {
        if (other.running.test()) {
            this->running.test_and_set();
        }
    }
    EventHandler& operator=(EventHandler&& other) = delete;
    ~EventHandler() {}

   public:
    bool allow_parallel_execution{false};
    std::source_location location;
    std::atomic_int missed_event_counter{0};
    std::atomic_flag running;
    std::function<void(const std::shared_ptr<Event>&)> callback;
};

class EventSystem;

template <std::derived_from<Event> EventType>
class EventContext {
   public:
    EventContext(EventSystem& system, const EventType& event) : system(system), event(event){};
    EventSystem& system;
    const EventType& event;
};

class EventSystem {
   public:
    EventSystem();
    EventSystem(const EventSystem& other) = delete;
    EventSystem& operator=(const EventSystem& other) = delete;
    EventSystem(EventSystem&& other) noexcept = delete;
    EventSystem& operator=(EventSystem&& other) = delete;
    ~EventSystem();

    /**
     * @brief Fires an event.
     *
     * This method fires an event to all registered event handler. Each event handler
     * will execute the event in a different worker thread, which means all event handler run in parallel. This method
     * does not block!
     *
     * Firing events is only possible when the handler registration is locked (after the baguette module setup is
     * complete)
     *
     * @tparam EventType The type of the event. Must be derived from Event
     * @param event
     */
    template <std::derived_from<Event> EventType>
    void fireEvent(EventType&& event) {
        std::type_index type(typeid(EventType));
        auto event_ptr = std::make_shared<EventType>(std::forward<EventType>(event));
        this->fireEvent(type, event_ptr);
    }

    void fireEvent(const std::shared_ptr<Event>& event);

    /**
     * @brief Registers and event handler
     *
     * Registers an event handler, which will be called when an event of the provided type is fired. Multiple event
     * handles of the same type are called in parallel, but a single event handler is only called from one thread at the
     * same time.
     *
     * Registering an event handler is only possible at the setup time of an baguette module. After that, every call to
     * registerEventHandler will result in an exception
     *
     * @tparam EventType The type of the event. Must be derived from Event
     * @tparam Callback The lambda which will execute the event handling
     * @param event_handler The callback. Must accept an const reference to EventContext<EventType>
     */
    template <std::derived_from<Event> EventType, typename Callback>
    void registerEventHandler(Callback event_handler, bool allow_parallel_execution = false,
                              std::source_location location = std::source_location::current())
        requires std::is_base_of_v<Event, EventType>
    {
        if (!freeze_registration.test()) {
            std::unique_lock lock(this->mutex);
            this->handler[std::type_index(typeid(EventType))].emplace_back(
                [this, event_handler = std::move(event_handler)](const std::shared_ptr<Event>& event) {
                    event_handler(EventContext<EventType>(*this, static_cast<const EventType&>(*event)));
                },
                allow_parallel_execution, location);
        } else {
            throw std::runtime_error("Event registration is only allowed at setup time");
        }
    }

    /**
     * @brief Execute a function in a worker thread
     *
     * @tparam Callable
     * @tparam Args
     * @param c the function the execute by the worker threads
     * @param args the arguments for the function. Can be omitted
     */
    template <typename Callable, typename... Args>
    void run(Callable&& c, Args&&... args) {
        this->thread_pool.detach_task(std::bind(std::forward<Callable>(c), std::forward<Args>(args)...));
    }

    void registerPolymorphicEventHandler(const std::type_info& type,
                                         std::function<void(const std::shared_ptr<Event>&)> event_handler,
                                         bool allow_parallel_execution = false,
                                         std::source_location location = std::source_location::current());

    void lockEventRegistration();

    void startTimerEvents();

    void startIOEvents();

    void stopIOEvents();

    void stopTimerEvents();

    asio::io_context& getIoContext();

   private:
    void fireEvent(const std::type_index& type, const std::shared_ptr<Event>& event);

   private:
    std::unique_ptr<asio::io_context> io_context;
    std::atomic_bool timer_events_running{false};
    std::atomic_flag freeze_registration;
    std::unordered_map<std::type_index, std::vector<EventHandler>> handler;
    mutable std::shared_mutex mutex;
    BS::thread_pool thread_pool;
    logger::Logger logger{"EventSystem"};
};

}  // namespace luhsoccer::event_system