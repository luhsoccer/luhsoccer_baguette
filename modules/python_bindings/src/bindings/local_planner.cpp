#include "bindings.hpp"

namespace luhsoccer::python {

using namespace local_planner;

template <>
void bindModule(py::module_& baguette_module, py::class_<LocalPlannerModule>& instance) {
    loadClassBindings<TaskData>(baguette_module, "TaskData");
    loadClassBindings<SimulationResult>(baguette_module, "SimulationResult");
    loadClassBindings<SimulationManager>(baguette_module, "SimulationManager");
    loadEnumBindings<LocalPlanner::LocalPlannerState>(baguette_module, "LocalPlannerState");
    instance.def("setTask", &LocalPlannerModule::setTask);
    instance.def("cancelTask", &LocalPlannerModule::cancelTask);
    instance.def("getState", &LocalPlannerModule::getState);
    instance.def("getSimulationManager", &LocalPlannerModule::getSimulationManager, py::return_value_policy::reference);
}

template <>
void bindEnum(py::enum_<LocalPlanner::LocalPlannerState>& instance) {
    instance.value("Created", LocalPlanner::LocalPlannerState::CREATED);
    instance.value("Error", LocalPlanner::LocalPlannerState::ERROR);
    instance.value("Idle", LocalPlanner::LocalPlannerState::IDLE);
    instance.value("Offline", LocalPlanner::LocalPlannerState::OFFLINE);
    instance.value("Running", LocalPlanner::LocalPlannerState::RUNNING);
}

template <>
void bindClass(py::class_<TaskData>& instance) {
    instance.def(py::init<const RobotIdentifier&>());
    /* TODO!!!! This does not work. Lists are empty when typ casted. @todo

    instance.def_readwrite("related_robots", &TaskData::related_robots);
    instance.def_readwrite("required_bools", &TaskData::required_bools);
    instance.def_readwrite("required_ints", &TaskData::required_ints);
    instance.def_readwrite("required_doubles", &TaskData::required_doubles);
    instance.def_readwrite("required_strings", &TaskData::required_strings);
    instance.def_readwrite("required_positions", &TaskData::required_positions);*/

    instance.def("addRelatedRobot",
                 [](TaskData& data, RobotIdentifier robot) { data.related_robots.push_back(robot); });
    instance.def("addBool", [](TaskData& data, bool b) { data.required_bools.push_back(b); });
    instance.def("addInt", [](TaskData& data, int i) { data.required_ints.push_back(i); });
    instance.def("addDouble", [](TaskData& data, double d) { data.required_doubles.push_back(d); });
    instance.def("addString",
                 [](TaskData& data, const std::string& string) { data.required_strings.push_back(string); });
    instance.def("addPosition", [](TaskData& data, const transform::Position& position) {
        data.required_positions.push_back(position);
    });
}

template <>
void bindClass(py::class_<SimulationResult>& instance) {
    instance.def_readonly("start_time", &SimulationResult::start_time);
    instance.def_readonly("end_time", &SimulationResult::end_time);
    instance.def_readonly("wm", &SimulationResult::wm, py::return_value_policy::copy);
}

template <>
void bindClass(py::class_<SimulationManager>& instance) {
    instance.def("runSyncSimulation", &SimulationManager::runSyncSimulation, py::arg(), py::arg(), py::arg(), py::arg(),
                 py::arg("wm") = nullptr);
}

}  // namespace luhsoccer::python