#include "bindings.hpp"

namespace luhsoccer::python {

using namespace ssl_interface;

template <>
void bindModule(py::module_& baguette_module, py::class_<SSLInterface>& instance) {
    loadEnumBindings<VisionDataSource>(baguette_module, "VisionDataSource");
    loadEnumBindings<GameControllerDataSource>(baguette_module, "GamecontrollerDataSource");
    loadEnumBindings<VisionPublishMode>(baguette_module, "VisionPublishMode");

    instance.def("setVisionDataSource", &SSLInterface::setVisionDataSource);
    instance.def("getVisionDataSource", &SSLInterface::getVisionDataSource);
    instance.def("setGameControllerDataSource", &SSLInterface::setGameControllerDataSource);
    instance.def("getGameControllerDataSource", &SSLInterface::getGameControllerDataSource);
    instance.def("setVisionPublishMode", &SSLInterface::setVisionPublishMode);
    instance.def("getVisionPublishMode", &SSLInterface::getVisionPublishMode);
}

template <>
void bindEnum(py::enum_<VisionDataSource>& instance) {
    instance.value("Disabled", VisionDataSource::DISABLED);
    instance.value("GameLog", VisionDataSource::GAME_LOG);
    instance.value("Network", VisionDataSource::NETWORK);
    instance.value("Simulation", VisionDataSource::SIMULATION);
}

template <>
void bindEnum(py::enum_<GameControllerDataSource>& instance) {
    instance.value("Disabled", GameControllerDataSource::DISABLED);
    instance.value("GameLog", GameControllerDataSource::GAME_LOG);
    instance.value("Internal", GameControllerDataSource::INTERNAL);
    instance.value("Network", GameControllerDataSource::NETWORK);
}

template <>
void bindEnum(py::enum_<VisionPublishMode>& instance) {
    instance.value("Disabled", VisionPublishMode::DISABLED);
    instance.value("Network", VisionPublishMode::NETWORK);
}

}  // namespace luhsoccer::python