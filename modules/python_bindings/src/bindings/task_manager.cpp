#include "bindings.hpp"

namespace luhsoccer::python {

using namespace task_manager;

template <>
void bindModule(nb::module_& /*baguette_module*/, nb::class_<TaskManager>& instance) {
    instance.def("updateTask", &TaskManager::updateTask, "skill"_a, "task_data"_a, "force"_a = false);
    instance.def("getLastSkill", &TaskManager::getLastSkill, "robot"_a);
    instance.def("getLastTaskData", &TaskManager::getLastTaskData, "robot"_a);
}

}  // namespace luhsoccer::python