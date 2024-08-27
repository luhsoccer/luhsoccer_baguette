#include "bindings.hpp"
#include "core/events.hpp"

namespace luhsoccer::python {

template <>
void bindClass(nb::class_<RobotIdentifier>& instance) {
    instance.def("getTeam", &RobotIdentifier::getTeam);
    instance.def("isAlly", &RobotIdentifier::isAlly);
    instance.def("isEnemy", &RobotIdentifier::isEnemy);
    instance.def("getFrame", &RobotIdentifier::getFrame);
    instance.def("__eq__",
                 [](const RobotIdentifier& instance, const RobotIdentifier& other) { return instance == other; });
    instance.def("__hash__", [](const RobotIdentifier& id) { return std::hash<RobotIdentifier>{}(id); });
    instance.def_static("createEmptyID", &RobotIdentifier::create_empty);

    addFormattedRepr(instance);
}

template <>
void bindDerivedClass(nb::class_<StopEvent, event_system::Event>& /*instance*/) {}

template <>
void bindDerivedClass(nb::class_<StartEvent, event_system::Event>& /*instance*/) {}

}  // namespace luhsoccer::python