#include "bindings.hpp"

namespace luhsoccer::python {

void createModuleBindings(py::module_& baguette_module, py::class_<baguette::TheBaguette>& wrapper,
                          baguette::TheBaguette& baguette) {
    // Here is the place to register more modules with python bindings
    loadModuleBindings(baguette_module, wrapper, *baguette.marker_service, "MarkerService");
    loadModuleBindings(baguette_module, wrapper, *baguette.ssl_interface, "SSLInterface");
    loadModuleBindings(baguette_module, wrapper, *baguette.game_data_provider, "GameDataProvider");
    loadModuleBindings(baguette_module, wrapper, *baguette.role_manager, "RoleManager");
    loadModuleBindings(baguette_module, wrapper, *baguette.robot_interface, "RobotInterface");
    loadModuleBindings(baguette_module, wrapper, *baguette.simulation_interface, "SimulationInterface");
    loadModuleBindings(baguette_module, wrapper, *baguette.local_planner, "LocalPlanner");
    loadModuleBindings(baguette_module, wrapper, *baguette.task_manager, "TaskManager");
}

}  // namespace luhsoccer::python
