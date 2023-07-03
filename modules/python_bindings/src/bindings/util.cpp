#include "bindings.hpp"
#include "utils/luts.hpp"

namespace luhsoccer::python {
using namespace util;

template <>
void bindClass(py::class_<Lut1D>& instance) {
    instance.def(py::init<>());
    instance.def("load", &Lut1D::load);
    instance.def("interpolate", &Lut1D::interpolate);
}

template <>
void bindClass(py::class_<Lut2D>& instance) {
    instance.def(py::init<>());
    instance.def("load", &Lut2D::load);
    instance.def("interpolate", &Lut2D::interpolate);
}

}  // namespace luhsoccer::python