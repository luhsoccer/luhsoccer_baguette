#include "local_planner/skills/component_uid.hpp"

namespace luhsoccer::local_planner {
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables) - necessary to get unique id
size_t ComponentUid::last_uid = 0;
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables) - necessary to get unique id
std::mutex ComponentUid::cookie_mtx;

ComponentUid::ComponentUid() {
    const std::lock_guard<std::mutex> lock(ComponentUid::cookie_mtx);
    // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer) - mutex have to be locked before
    this->uid = ComponentUid::last_uid++;
}

}  // namespace luhsoccer::local_planner