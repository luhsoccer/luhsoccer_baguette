#include "local_planner/skills/task.hpp"

namespace luhsoccer::local_planner {

bool TaskData::setCookie(const ComponentUid& component_uid, const std::string& key, const std::any& value) const {
    // check if cookie space exists
    auto cookie_space_it = this->cookie_jar.find(component_uid);
    if (cookie_space_it == this->cookie_jar.end()) {
        this->cookie_jar[component_uid] = std::map<std::string, std::any>();
        cookie_space_it = this->cookie_jar.find(component_uid);
    }
    cookie_space_it->second[key] = value;
    return true;
}
}  // namespace luhsoccer::local_planner