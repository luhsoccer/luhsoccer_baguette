#include "bindings.hpp"
#include "observer/continuous_observer.hpp"

namespace luhsoccer::python {

using namespace task_manager;

template <>
void bindModule(py::module_& /*baguette_module*/, py::class_<TaskManager>& instance) {
    instance.def("addRoleCallback", [](TaskManager& instance, const std::string& role, TaskCallback& callback) {
        instance.registerCallback(role, [callback, role](const std::vector<transform::RobotHandle>& robots,
                                                         const std::shared_ptr<const observer::Observer>& observer,
                                                         const std::shared_ptr<const transform::WorldModel>& wm) {
            return executePythonCallbackWithErrorLogging("TaskManagerCallback[Role=" + role + "]",
                                                         [&]() { return callback(robots, observer, wm); });
        });
    });
}

}  // namespace luhsoccer::python