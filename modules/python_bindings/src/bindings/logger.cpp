#include "bindings.hpp"

namespace luhsoccer::python {

using namespace logger;

namespace {
CustomSourceLog getSourceLocationFromPython(const nb::module_& inspect_module) {
    auto get_frame_info = inspect_module.attr("getframeinfo");
    auto current_frame = inspect_module.attr("currentframe");
    auto frame_info = get_frame_info(current_frame());

    return {nb::cast<std::string>(frame_info.attr("filename")), nb::cast<int>(frame_info.attr("lineno"))};
}
}  // namespace

template <>
void bindClass(nb::class_<Logger>& instance) {
    nb::module_ inspect = nb::module_::import_("inspect");

    instance.def(nb::init<std::string>());
    instance.def("trace", [=](Logger& self, std::string_view message) {
        self.logImpl(getSourceLocationFromPython(inspect), LogLevel::TRACE, message);
    });
    instance.def("debug", [=](Logger& self, std::string_view message) {
        self.logImpl(getSourceLocationFromPython(inspect), LogLevel::DEBUG, message);
    });
    instance.def("info", [=](Logger& self, std::string_view message) {
        self.logImpl(getSourceLocationFromPython(inspect), LogLevel::INFO, message);
    });
    instance.def("warning", [=](Logger& self, std::string_view message) {
        self.logImpl(getSourceLocationFromPython(inspect), LogLevel::WARNING, message);
    });
    instance.def("error", [=](Logger& self, std::string_view message) {
        self.logImpl(getSourceLocationFromPython(inspect), LogLevel::ERROR, message);
    });
}

}  // namespace luhsoccer::python