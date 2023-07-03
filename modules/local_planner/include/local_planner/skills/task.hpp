

#pragma once

#include "transform/position.hpp"
#include "component_uid.hpp"

#include <any>
#include <map>
#include <utility>

namespace luhsoccer::local_planner {

class AbstractComponent;
/**
 * @brief data that has to be provided along with a skill pointer to form a task
 * @warning Do not use the same @class object in two different threads, copy if necessary
 */
struct TaskData {
    friend AbstractComponent;

    explicit TaskData(const RobotIdentifier& robot, std::vector<RobotIdentifier> related_robots = {},
                      std::vector<transform::Position> required_positions = {}, std::vector<int> required_ints = {},
                      std::vector<double> required_doubles = {}, std::vector<bool> required_bools = {},
                      std::vector<std::string> required_strings = {})
        : robot(robot),
          related_robots(std::move(related_robots)),
          required_positions(std::move(required_positions)),
          required_ints(std::move(required_ints)),
          required_doubles(std::move(required_doubles)),
          required_bools(std::move(required_bools)),
          required_strings(std::move(required_strings)){};
    RobotIdentifier robot;
    std::vector<RobotIdentifier> related_robots;
    std::vector<transform::Position> required_positions;
    std::vector<int> required_ints;
    std::vector<double> required_doubles;
    std::vector<bool> required_bools;
    std::vector<std::string> required_strings;

    bool setCookie(const ComponentUid& component_uid, const std::string& key, const std::any& value) const;

    template <typename T>
    std::optional<T> getCookie(const ComponentUid& component_uid, const std::string& key) const {
        auto cookie_space_it = this->cookie_jar.find(component_uid);
        if (cookie_space_it == this->cookie_jar.end()) return std::nullopt;

        auto cookie_it = cookie_space_it->second.find(key);
        if (cookie_it == cookie_space_it->second.end()) return std::nullopt;

        try {
            return std::any_cast<T>(cookie_it->second);
        } catch (const std::bad_any_cast& e) {
            LOG_WARNING(
                logger::Logger{"Component"},
                "Component with id {:d} tries to get Cookie '{}' wit type '{}' but the stored cookie has type '{}'",
                component_uid, key, typeid(T).name(), cookie_it->second.type().name());
            return std::nullopt;
        }
    }

   private:
    mutable std::map<size_t, std::map<std::string, std::any>> cookie_jar{};
};
}  // namespace luhsoccer::local_planner
