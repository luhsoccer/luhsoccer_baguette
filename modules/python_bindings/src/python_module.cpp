#include "module.hpp"
#include "baguette.hpp"
#include "bindings.hpp"

// Disable the checks here since the code is external and included via a macro
// NOLINTBEGIN
PYBIND11_MODULE(_baguette_py, module) {
    static luhsoccer::baguette::TheBaguette baguette;
    module.doc() = "The python module for controlling the baguette";

    luhsoccer::python::loadBindings(module, baguette);
}
// NOLINTEND