#include "bindings.hpp"
#include "oneapi/tbb/concurrent_queue.h"

namespace luhsoccer::python {

using namespace event_system;

class PyEvent : public Event {
   public:
    PyEvent() = default;
    PyEvent(const PyEvent&) = default;
    PyEvent(PyEvent&&) = delete;
    PyEvent& operator=(const PyEvent&) = default;
    PyEvent& operator=(PyEvent&&) = delete;
    virtual ~PyEvent() = default;
};

struct EventType {
    // NOLINTNEXTLINE(readability-identifier-naming) name is given by nanobind
    static constexpr auto Name = nb::detail::const_name("Type[EventType]");
};

struct EventHandlerType {
    // NOLINTNEXTLINE(readability-identifier-naming) name is given by nanobind
    static constexpr auto Name = nb::detail::const_name("Callable[[EventType], None]");
};

struct EventQueue {
    oneapi::tbb::concurrent_bounded_queue<std::shared_ptr<Event>> eventQueue{};

    EventQueue() { this->eventQueue.set_capacity(512); }
};

template <>
void bindTool(nb::module_& baguette_module, nb::class_<EventSystem>& instance) {
    instance.def(
        "fireEvent", [](EventSystem& self, const std::shared_ptr<Event>& event) { self.fireEvent(event); },
        nb::call_guard<nb::gil_scoped_release>());
    loadClassBindings<EventQueue>(baguette_module, "EventQueue");

    loadClassBindings<Event>(baguette_module, "Event", nb::is_final());
    loadDerivedClassBindings<PyEvent, Event>(baguette_module, "PyEvent");

    loadDerivedClassBindings<TimerEvent, Event>(baguette_module, "TimerEvent");
    loadDerivedClassBindings<TimerEvent100Hz, TimerEvent>(baguette_module, "TimerEvent100Hz");
    loadDerivedClassBindings<TimerEvent50Hz, TimerEvent>(baguette_module, "TimerEvent50Hz");
    loadDerivedClassBindings<TimerEvent10Hz, TimerEvent>(baguette_module, "TimerEvent10Hz");
    loadDerivedClassBindings<TimerEvent5Hz, TimerEvent>(baguette_module, "TimerEvent5Hz");
    loadDerivedClassBindings<TimerEvent1Hz, TimerEvent>(baguette_module, "TimerEvent1Hz");
    loadDerivedClassBindings<TimerEvent2Sec, TimerEvent>(baguette_module, "TimerEvent2Sec");
    loadDerivedClassBindings<TimerEvent5Sec, TimerEvent>(baguette_module, "TimerEvent5Sec");
    loadDerivedClassBindings<TimerEvent10Sec, TimerEvent>(baguette_module, "TimerEvent10Sec");
}

template <>
void bindClass(nb::class_<EventQueue>& instance) {
    instance.def(nb::init<>());
    instance.def("enableReceivingFor",
                 [](EventQueue& self, EventSystem& event_system, nb::typed<nb::type_object, EventType>& type) {
                     // This check if the type was registered by us or is a subclass from our event type.
                     if (nb::detail::nb_type_check(type.value.ptr())) {
                         // Returns the c++ type or the c++ base type if the type is a subclass of the registered type.
                         // Should never fail since we checked it before.
                         auto cpp_type = nb::detail::nb_type_info(type.value.ptr());
                         auto name = nb::steal<nb::str>(nb::detail::nb_type_name(type.value.ptr()));

                         event_system.registerPolymorphicEventHandler(
                             *cpp_type,
                             [&self](const std::shared_ptr<Event>& event) {
                                 if (!self.eventQueue.try_emplace(event)) {
                                     static logger::Logger logger("PythonEventQueue");
                                     logger.warning(
                                         "EventQueue is full, dropping event {}. This is necessary to keep baguette "
                                         "running smoothly.");
                                 }
                             },
                             true);  // We allow parallel execution since we are using a concurrent queue

                     } else {
                         throw std::invalid_argument("Wrong type submitted");
                     }
                 });

    instance.def(
        "pop",
        [](EventQueue& self) {
            std::shared_ptr<Event> event;
            self.eventQueue.pop(event);
            return event;
        },
        nb::call_guard<nb::gil_scoped_release>());

    instance.def(
        "enqueue", [](EventQueue& self, const std::shared_ptr<Event>& event) { self.eventQueue.emplace(event); },
        nb::call_guard<nb::gil_scoped_release>());
}

template <>
void bindClass(nb::class_<Event>& instance) {
    instance.def_ro("created_at", &Event::created_at);
}

template <>
void bindDerivedClass(nb::class_<PyEvent, Event>& instance) {
    instance.def(nb::init<>());
}

template <>
void bindDerivedClass(nb::class_<TimerEvent, Event>& instance) {
    instance.def("getPeriod", &TimerEvent::getPeriod);
}

template <>
void bindDerivedClass(nb::class_<TimerEvent100Hz, TimerEvent>& /*instance*/) {}

template <>
void bindDerivedClass(nb::class_<TimerEvent50Hz, TimerEvent>& /*instance*/) {}

template <>
void bindDerivedClass(nb::class_<TimerEvent10Hz, TimerEvent>& /*instance*/) {}

template <>
void bindDerivedClass(nb::class_<TimerEvent5Hz, TimerEvent>& /*instance*/) {}

template <>
void bindDerivedClass(nb::class_<TimerEvent1Hz, TimerEvent>& /*instance*/) {}

template <>
void bindDerivedClass(nb::class_<TimerEvent2Sec, TimerEvent>& /*instance*/) {}

template <>
void bindDerivedClass(nb::class_<TimerEvent5Sec, TimerEvent>& /*instance*/) {}

template <>
void bindDerivedClass(nb::class_<TimerEvent10Sec, TimerEvent>& /*instance*/) {}

}  // namespace luhsoccer::python
