#pragma once

#include "nanobind/nanobind.h"
#include "nanobind/stl/chrono.h"

#include "time/time.hpp"

namespace nanobind::detail {

template <>
class type_caster<luhsoccer::time::TimePoint> : public duration_caster<luhsoccer::time::TimePoint> {};

template <>
class type_caster<luhsoccer::time::Duration> : public duration_caster<luhsoccer::time::Duration> {};

}  // namespace nanobind::detail