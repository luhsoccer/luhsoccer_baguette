#include "core/common_types.hpp"
#include "core/module.hpp"
#include "core/robot_identifier.hpp"

#include <thread>
#include <fmt/core.h>

namespace luhsoccer {

void BaguetteModule::loop([[maybe_unused]] std::atomic_bool& should_run) { this->is_running = false; };

std::string_view format_as(const Team& team) {
    switch (team) {
        case Team::ALLY:
            return "Ally";
        case Team::ENEMY:
            return "Enemy";
    }
}

std::string_view format_as(const TeamColor& team) {
    switch (team) {
        case TeamColor::BLUE:
            return "Blue";
        case TeamColor::YELLOW:
            return "Yellow";
    }
}

[[nodiscard]] std::string RobotIdentifier::getFrame() const {
    if (create_empty() == *this) {
        return "invalid";
    }
    // TODO maybe store frame?
    return isAlly() ? "robot_ally_" + std::to_string(id) : "robot_enemy_" + std::to_string(id);
}

std::string format_as(RobotIdentifier handle) {
    if (RobotIdentifier::create_empty() == handle) {
        return fmt::format("Invalid");
    }
    if (handle.team == Team::ALLY) {
        return fmt::format("AllyRobot{:02d}", handle.id);
    } else {
        return fmt::format("EnemyRobot{:02d}", handle.id);
    }
}

}  // namespace luhsoccer