#include "bindings.hpp"

namespace luhsoccer::python {

void createModuleBindings(nb::module_& baguette_module, nb::class_<baguette::TheBaguette>& wrapper) {
    // Here is the place to register more modules with python bindings
    loadModuleBindings(baguette_module, wrapper, &baguette::TheBaguette::marker_service, "MarkerService",
                       "marker_service");
    loadModuleBindings(baguette_module, wrapper, &baguette::TheBaguette::ssl_interface, "SSLInterface",
                       "ssl_interface");
    loadModuleBindings(baguette_module, wrapper, &baguette::TheBaguette::game_data_provider, "GameDataProvider",
                       "game_data_provider");
    loadModuleBindings(baguette_module, wrapper, &baguette::TheBaguette::robot_interface, "RobotInterface",
                       "robot_interface");
    loadModuleBindings(baguette_module, wrapper, &baguette::TheBaguette::simulation_interface, "SimulationInterface",
                       "simulation_interface");
    loadModuleBindings(baguette_module, wrapper, &baguette::TheBaguette::robot_control_module, "RobotControlModule",
                       "robot_control_module");
    loadModuleBindings(baguette_module, wrapper, &baguette::TheBaguette::task_manager, "TaskManager", "task_manager");
    loadModuleBindings(baguette_module, wrapper, &baguette::TheBaguette::software_manager, "SoftwareManager",
                       "software_manager");
}

}  // namespace luhsoccer::python
