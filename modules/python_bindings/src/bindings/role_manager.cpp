#include "bindings.hpp"
#include "observer/continuous_observer.hpp"

namespace luhsoccer::python {

using namespace role_manager;

class PyRoleProvider : public RoleProvider {
   public:
    using RoleProvider::RoleProvider;

    [[nodiscard]] std::vector<std::string> getPossibleRoles() const override {
        return executePythonCallbackWithErrorLogging("getPossibleRoles", [&]() {
            PYBIND11_OVERRIDE_PURE(std::vector<std::string>, RoleProvider, getPossibleRoles);
        });
    }

    std::vector<std::pair<transform::RobotHandle, std::string>> provideRoles(
        std::vector<transform::RobotHandle> ally_robots, std::vector<transform::RobotHandle> enemy_robots,
        std::shared_ptr<const observer::Observer> observer) override {
        std::vector<std::pair<transform::RobotHandle, std::string>> map_type;

        return executePythonCallbackWithErrorLogging("provideRoles", [&]() {
            PYBIND11_OVERRIDE_PURE(decltype(map_type), RoleProvider, provideRoles, ally_robots, enemy_robots, observer);
        });
    }
};

template <>
void bindModule(py::module_& baguette_module, py::class_<RoleManager>& instance) {
    loadVirtualClassBindings<RoleProvider, PyRoleProvider>(baguette_module, "RoleProvider");

    instance.def("setRoleProvider", &RoleManager::setRoleProvider);
}

template <>
void bindVirtualClass(py::class_<RoleProvider, PyRoleProvider, std::shared_ptr<RoleProvider>>& instance) {
    instance.def(py::init<>());
    instance.def("getPossibleRoles", &RoleProvider::getPossibleRoles);
    instance.def("provideRoles", &RoleProvider::provideRoles);
}

}  // namespace luhsoccer::python