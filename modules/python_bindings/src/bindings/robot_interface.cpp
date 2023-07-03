#include "bindings.hpp"

namespace luhsoccer::python {

using namespace robot_interface;

template <>
void bindModule(py::module_& baguette_module, py::class_<RobotInterface>& instance) {
    loadEnumBindings<RobotConnection>(baguette_module, "RobotConnection");
    instance.def("setConnectionType", &RobotInterface::setConnectionType);
    instance.def("getConnectionType", &RobotInterface::getConnectionType);
}

template <>
void bindEnum(py::enum_<RobotConnection>& instance) {
    instance.value("Disabled", RobotConnection::DISABLED);
    instance.value("Network", RobotConnection::NETWORK);
    instance.value("Serial", RobotConnection::SERIAL);
    instance.value("SerialLegacy", RobotConnection::SERIAL_LEGACY);
    instance.value("Simulation", RobotConnection::SIMULATION);
    instance.value("SimulationLegacy", RobotConnection::SIMULATION_LEGACY);
}

}  // namespace luhsoccer::python