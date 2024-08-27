#include "bindings.hpp"

namespace luhsoccer::python {

using namespace robot_control;

template <>
void bindModule(nb::module_& baguette_module, nb::class_<RobotControlModule>& instance) {
    loadClassBindings<TaskData>(baguette_module, "TaskData");

    loadEnumBindings<RobotControllerState>(baguette_module, "RobotControllerState");
    instance.def("setTask", &RobotControlModule::setTask);
    instance.def("cancelTask", &RobotControlModule::cancelTask);
    instance.def("getState", &RobotControlModule::getState);
}

template <>
void bindClass(nb::class_<TaskData>& instance) {
    instance.def(nb::init<const RobotIdentifier&>());
    /* TODO!!!! This does not work. Lists are empty when typ casted. @todo

    instance.def_readwrite("related_robots", &TaskData::related_robots);
    instance.def_readwrite("required_bools", &TaskData::required_bools);
    instance.def_readwrite("required_ints", &TaskData::required_ints);
    instance.def_readwrite("required_doubles", &TaskData::required_doubles);
    instance.def_readwrite("required_strings", &TaskData::required_strings);
    instance.def_readwrite("required_positions", &TaskData::required_positions);*/

    instance.def("setRobot", [](TaskData& data, RobotIdentifier robot) { data.robot = robot; });
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

}  // namespace luhsoccer::python