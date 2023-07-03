#pragma once

#include <mutex>
#include "local_planner/skills/task.hpp"
#include "local_planner/skills/component_uid.hpp"
#include <variant>

namespace luhsoccer::marker {
class Marker;
class LinePlot;
}  // namespace luhsoccer::marker
namespace luhsoccer::local_planner {

using Marker = std::variant<marker::Marker, marker::LinePlot>;

class AbstractComponent {
   public:
    AbstractComponent() = default;
    AbstractComponent& operator=(const AbstractComponent& other) {
        if (this != &other) {
            this->uid = ComponentUid();
        }
        return *this;
    };
    AbstractComponent& operator=(AbstractComponent&&) = default;
    AbstractComponent(const AbstractComponent&) : uid(){};
    AbstractComponent(AbstractComponent&&) = default;

    virtual ~AbstractComponent() = default;

    [[nodiscard]] size_t getUid() const { return uid; }

   protected:
    // NOLINTNEXTLINE(modernize-use-nodiscard) - changes mutable
    bool setCookie(const TaskData& td, const std::string& key, const std::any& value) const {
        return td.setCookie(this->uid, key, value);
    }

    template <typename T>
    std::optional<T> getCookie(const TaskData& td, const std::string& key) const {
        return td.getCookie<T>(this->uid, key);
    }
    [[nodiscard]] ComponentUid getComponentUid() const { return this->uid; }

   private:
    ComponentUid uid;
};

}  // namespace luhsoccer::local_planner