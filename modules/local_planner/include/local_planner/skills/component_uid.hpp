#pragma once

#include <cstddef>
#include <mutex>
namespace luhsoccer::local_planner {

class AbstractComponent;

class ComponentUid {
   public:
    operator size_t() const { return this->uid; }

   private:
    // NOLINTNEXTLINE(modernize-use-equals-delete)
    ComponentUid();
    size_t uid;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables) - necessary to get unique id
    static std::mutex cookie_mtx;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables) - necessary to get unique id
    static size_t last_uid;

    friend AbstractComponent;
};

}  // namespace luhsoccer::local_planner