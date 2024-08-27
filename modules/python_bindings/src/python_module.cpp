#include "core/module.hpp"
#include "baguette.hpp"
#include "bindings.hpp"

namespace luhsoccer::python {
void loadBindings(nb::module_& baguette_module) {
    nb::class_<baguette::TheBaguette> baguette(baguette_module, "NativeBaguette");

    createToolBindings(baguette_module, baguette);
    createModuleBindings(baguette_module, baguette);

    // Register start function for the python object
    // TODO needs more investigation but does not work currently
    // baguette.def(nb::init<>());
    baguette.def_static("stop", &baguette::TheBaguette::signalHandler, nb::call_guard<nb::gil_scoped_release>(),
                        nb::arg("signal") = 0);
    baguette.def(
        "getInstance",
        []() -> baguette::TheBaguette& {
            static baguette::TheBaguette baguette;
            return baguette;
        },
        nb::rv_policy::reference);
    baguette.def(
        "start",
        [&](baguette::TheBaguette& self, std::function<void()>& setup_function) {
            nb::module_::import_("atexit").attr("register")(nb::cpp_function([&]() {
                nb::gil_scoped_release release;
                self.stop();
            }));

            // Don't start the instance twice
            if (!self.started) {
                self.load(true);
                executePythonCallbackWithErrorLogging("setup", [&setup_function]() { setup_function(); });

                nb::gil_scoped_release release;
                self.run();
            }
        },
        nb::arg("setup_function"));
}
}  // namespace luhsoccer::python

// NOLINTBEGIN Disable the checks here since the code is external and included via a macro
NB_MODULE(_baguette_py, module) {
    module.doc() = R"(
        The python module for controlling the baguette
        
        %%EventType = TypeVar('EventType', bound='Event')
    )";

    luhsoccer::python::loadBindings(module);
}
// NOLINTEND