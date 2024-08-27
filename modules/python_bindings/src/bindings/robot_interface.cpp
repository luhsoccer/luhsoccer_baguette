#include "bindings.hpp"

namespace luhsoccer::python {

using namespace robot_interface;

template <>
void bindModule(nb::module_& baguette_module, nb::class_<RobotInterface>& instance) {
    loadEnumBindings<RobotConnection>(baguette_module, "RobotConnection");
    instance.def("setConnectionType", &RobotInterface::setConnectionType);
    instance.def("getConnectionType", &RobotInterface::getConnectionType);
}

}  // namespace luhsoccer::python