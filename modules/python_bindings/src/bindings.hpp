#pragma once

#include "baguette.hpp"
#include "id_provider.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <condition_variable>

namespace luhsoccer::python {

namespace py = pybind11;

template <typename T>
void bindModule(py::module_& baguette_module, py::class_<T>& instance);

template <typename T>
void bindEnum(py::enum_<T>& instance);

template <typename T>
void bindClass(py::class_<T>& instance);

template <typename T, typename P>
void bindDerivedClass(py::class_<T, P>& instance);

template <typename T>
void bindSharedClass(py::class_<T, std::shared_ptr<T>>& instance);

template <typename T, typename D>
void bindVirtualClass(py::class_<T, D, std::shared_ptr<T>>& instance);

template <typename T>
inline void addFormattedRepr(py::class_<T>& instance) {
    instance.def("__repr__", [](const T& object) {
        std::stringstream s;
        s << object;
        return s.str();
    });
}

template <typename T>
inline void loadModuleBindings(py::module_& baguette_module, py::class_<baguette::TheBaguette>& baguette_class,
                               T& module, std::string_view name) {
    py::class_<T> instance(baguette_module, name.data());
    baguette_class.def_property_readonly(
        module.moduleName().data(), [&module](py::object& /*self*/) { return &module; },
        py::return_value_policy::reference);
    bindModule(baguette_module, instance);
}

template <typename T>
inline void loadEnumBindings(py::module_& baguette_module, std::string_view name) {
    py::enum_<T> instance(baguette_module, name.data());
    bindEnum(instance);
}

template <typename T>
inline void loadClassBindings(py::module_& baguette_module, std::string_view name) {
    py::class_<T> instance(baguette_module, name.data());
    bindClass(instance);
}

template <typename T, typename P>
inline void loadDerivedClassBindings(py::module_& baguette_module, std::string_view name) {
    py::class_<T, P> instance(baguette_module, name.data());
    bindDerivedClass(instance);
}

template <typename T>
inline void loadSharedClassBindings(py::module_& baguette_module, std::string_view name) {
    py::class_<T, std::shared_ptr<T>> instance(baguette_module, name.data());
    bindSharedClass(instance);
}

template <typename T, typename D>
inline void loadVirtualClassBindings(py::module_& baguette_module, std::string_view name) {
    py::class_<T, D, std::shared_ptr<T>> instance(baguette_module, name.data());
    bindVirtualClass(instance);
}

void createModuleBindings(py::module_& baguette_module, py::class_<baguette::TheBaguette>& wrapper,
                          baguette::TheBaguette& baguette);

void createToolBindings(py::module_& baguette_module, py::class_<baguette::TheBaguette>& wrapper,
                        baguette::TheBaguette& baguette);

inline void loadBindings(py::module_& baguette_module, baguette::TheBaguette& instance) {
    py::class_<baguette::TheBaguette> baguette(baguette_module, "Baguette");

    // Register start function for the python object
    baguette.def("start", [&](baguette::TheBaguette& self) {
        // Don't start the instance twice
        if (!self.started) {
            // Start run in a custom thread to avoid the python GIL
            std::atomic_bool loaded = false;
            std::thread([&]() {
                self.load(false);
                loaded = true;
                self.run();
            }).detach();

            while (!loaded) {  // wait until the main thread has loaded the modules, so they can accessed by python
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }

        py::module_::import("atexit").attr("register")(py::cpp_function([&]() {
            py::gil_scoped_release release;
            self.stop();
        }));
    });

    baguette.def("isRunning", &baguette::TheBaguette::isRunning);

    /*baguette.def("stop", [&](baguette::TheBaguette& self) {

    });*/

    baguette_module.def(
        "getInstance", [&instance]() { return &instance; }, py::return_value_policy::reference);

    createToolBindings(baguette_module, baguette, instance);
    createModuleBindings(baguette_module, baguette, instance);
}

void bindConfigs(py::module_& baguette_module);

template <typename T>
auto executePythonCallbackWithErrorLogging(const std::string& callback_name, T callback) {
    try {
        return callback();
    } catch (py::error_already_set& e) {
        LOG_ERROR(logger::Logger("python_bindings"), "Error while executing python callback '{}':\n{}", callback_name,
                  e.what());
        throw std::runtime_error("Error while executing python callback");
    } catch (py::cast_error& e) {
        LOG_ERROR(logger::Logger("python_bindings"), "Error while casting types from python callback '{}':\n{}",
                  callback_name, e.what());
        throw std::runtime_error("Error while executing python callback");
    } catch (...) {
        LOG_ERROR(logger::Logger("python_bindings"), "Error while executing python callback: '{}'", callback_name);
        throw std::runtime_error("Error while executing python callback");
    }
}

}  // namespace luhsoccer::python