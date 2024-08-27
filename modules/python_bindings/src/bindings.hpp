#pragma once

#include "baguette.hpp"
#include "id_provider.hpp"
#include "nanobind/nanobind.h"
#include "nanobind/stl/function.h"
#include "nanobind/stl/string.h"
#include "nanobind/stl/string_view.h"
#include "nanobind/stl/vector.h"
#include "nanobind/stl/shared_ptr.h"
#include "nanobind/stl/unique_ptr.h"
#include "nanobind/stl/optional.h"
#include "nanobind/stl/chrono.h"
#include "nanobind/stl/pair.h"
#include "nanobind/stl/unordered_map.h"
#include "nanobind/stl/map.h"
#include "nanobind/stl/variant.h"
#include "nanobind/stl/array.h"
#include "nanobind/eigen/dense.h"
#include "nanobind/operators.h"
#include "nanobind/trampoline.h"

#include "type_casters/time.hpp"

#include "magic_enum.hpp"

namespace luhsoccer::python {

namespace nb = nanobind;

using namespace nb::literals;

template <typename T>
auto executePythonCallbackWithErrorLogging(const std::string& callback_name, T callback) {
    try {
        return callback();
    } catch (nb::python_error& e) {
        logger::Logger("python_bindings")
            .error("Error while executing python callback '{}':\n{}", callback_name, e.what());
        nb::raise_from(e, PyExc_RuntimeError, "Error while executing python callback");
    } catch (nb::cast_error& e) {
        logger::Logger("python_bindings")
            .error("Error while casting types from python callback '{}':\n{}", callback_name, e.what());
        throw std::runtime_error("Error while executing python callback");
    } catch (...) {
        logger::Logger("python_bindings").error("Error while executing python callback: '{}'", callback_name);
        throw std::runtime_error("Error while executing python callback");
    }
}

template <typename T>
void bindClass(nb::class_<T>& instance);

template <typename T, typename P>
void bindDerivedClass(nb::class_<T, P>& instance);

template <typename T>
void bindModule(nb::module_& baguette_module, nb::class_<T>& instance);

template <typename T>
void bindTool(nb::module_& baguette_module, nb::class_<T>& instance);

template <typename T>
void addFormattedRepr(nb::class_<T>& instance) {
    instance.def("__repr__", [](const T& object) { return fmt::to_string(object); });
}

template <typename T>
void loadModuleBindings(nb::module_& baguette_module, nb::class_<baguette::TheBaguette>& baguette_class,
                        std::unique_ptr<T> baguette::TheBaguette::*module, std::string_view name,
                        const std::string& field_name) {
    nb::class_<T> instance(baguette_module, name.data());
    baguette_class.def_prop_ro(
        field_name.c_str(), [module](baguette::TheBaguette& self) -> const T& { return *(self.*module); },
        nb::rv_policy::reference_internal);
    bindModule(baguette_module, instance);
}

template <typename T>
void loadToolBindings(nb::module_& baguette_module, std::string_view name) {
    nb::class_<T> instance(baguette_module, name.data());
    bindTool(baguette_module, instance);
}

// TODO: This could be modified in a future version to comply with pythons UPPER_CASE naming convention
// but that would be a breaking change
inline std::string pythonEnumName(std::string_view name_view) {
    std::string result;
    result.reserve(name_view.size());
    bool upper = true;
    for (char c : name_view) {
        if (c == '_') {
            upper = true;
        } else if (upper) {
            result.push_back(std::toupper(c));
            upper = false;
        } else {
            result.push_back(std::tolower(c));
        }
    }
    return result;
}

/**
 * @brief Auto generates bindings for an enum class.
 * This function generates bindings for all variants of an given enum.
 *
 * @tparam T The enum
 * @param baguette_module the baguette python module
 * @param name the name of the enum
 */
template <typename T>
void loadEnumBindings(nb::module_& baguette_module, std::string_view name) {
    nb::enum_<T> instance(baguette_module, name.data());
    for (auto value : magic_enum::enum_values<T>()) {
        instance.value(pythonEnumName(magic_enum::enum_name(value)).c_str(), value);
    }
}

template <typename T, typename... Extras>
void loadClassBindings(nb::module_& baguette_module, std::string_view name, const Extras&... ex) {
    nb::class_<T> instance(baguette_module, name.data(), ex...);
    bindClass(instance);
}

template <typename T, typename P>
void loadDerivedClassBindings(nb::module_& baguette_module, std::string_view name) {
    nb::class_<T, P> instance(baguette_module, name.data());
    bindDerivedClass(instance);
}

void createModuleBindings(nb::module_& baguette_module, nb::class_<baguette::TheBaguette>& wrapper);

void createToolBindings(nb::module_& baguette_module, nb::class_<baguette::TheBaguette>& wrapper);

void bindConfigs(nb::module_& baguette_module);

}  // namespace luhsoccer::python