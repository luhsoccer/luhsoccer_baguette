#include "bindings.hpp"

namespace luhsoccer::python {
using namespace time;

template <>
void bindClass(py::class_<Duration>& instance) {
    instance.def(py::init<double>());
    instance.def("asSec", &Duration::asSec);
    instance.def("asNSec", &Duration::asNSec);
    instance.def(py::self + py::self);
    instance.def(py::self += py::self);
    instance.def(py::self - py::self);
    instance.def(py::self -= py::self);
    addFormattedRepr(instance);
}

template <>
void bindClass(py::class_<TimePoint>& instance) {
    instance.def(py::init<double>());
    instance.def("asSec", &TimePoint::asSec);
    instance.def("asNSec", &TimePoint::asNSec);
    addFormattedRepr(instance);
}

}  // namespace luhsoccer::python