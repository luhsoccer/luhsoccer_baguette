#include "bindings.hpp"

namespace luhsoccer::python {

using namespace software_manager;

template <>
void bindModule(nb::module_& baguette_module, nb::class_<SoftwareManager>& instance) {
    loadEnumBindings<software_manager::SoftwareComponent>(baguette_module, "SoftwareComponent");

    instance.def("getComponents", &SoftwareManager::getComponents);
    instance.def("startComponent", &SoftwareManager::startComponent);
    instance.def("stopComponent", &SoftwareManager::stopComponent);
    instance.def("isRunning", &SoftwareManager::isRunning);
    instance.def("shouldRun", &SoftwareManager::shouldRun);
}

}  // namespace luhsoccer::python