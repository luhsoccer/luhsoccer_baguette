#include "bindings.hpp"

namespace luhsoccer::python {

template <>
void bindClass(py::class_<RobotIdentifier>& instance) {
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
void bindEnum(py::enum_<Team>& instance) {
    instance.value("Ally", Team::ALLY);
    instance.value("Enemy", Team::ENEMY);
}

template <>
void bindEnum(py::enum_<TeamColor>& instance) {
    instance.value("Blue", TeamColor::BLUE);
    instance.value("Yellow", TeamColor::YELLOW);
}

}  // namespace luhsoccer::python