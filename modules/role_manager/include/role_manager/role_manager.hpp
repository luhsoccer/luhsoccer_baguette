#pragma once

#include <unordered_set>
#include <vector>
#include "module.hpp"
#include "robot_identifier.hpp"
#include <transform/handles.hpp>
#include "time/time.hpp"

namespace luhsoccer::game_data_provider {
class GameDataProvider;
}

namespace luhsoccer::observer {
class Observer;
}

namespace luhsoccer::marker {
class MarkerService;
}

namespace luhsoccer::role_manager {

class RoleProvider {
   public:
    RoleProvider() = default;
    RoleProvider(const RoleProvider&) = default;
    RoleProvider(RoleProvider&&) = default;
    RoleProvider& operator=(const RoleProvider&) = default;
    RoleProvider& operator=(RoleProvider&&) = default;
    virtual ~RoleProvider() = default;

    [[nodiscard]] virtual std::vector<std::string> getPossibleRoles() const = 0;
    virtual std::vector<std::pair<transform::RobotHandle, std::string>> provideRoles(
        std::vector<transform::RobotHandle> ally_robots, std::vector<transform::RobotHandle> enemy_robots,
        std::shared_ptr<const observer::Observer> observer) = 0;
};

class RoleManager : public BaguetteModule {
   public:
    RoleManager(game_data_provider::GameDataProvider&, marker::MarkerService&);
    RoleManager(const RoleManager&) = delete;
    RoleManager(RoleManager&&) = delete;
    RoleManager& operator=(const RoleManager&) = delete;
    RoleManager& operator=(RoleManager&&) = delete;
    virtual ~RoleManager() = default;

    void loop(std::atomic_bool& should_run) override;

    constexpr std::string_view moduleName() override { return "role_manager"; }

    void setRoleProvider(std::shared_ptr<RoleProvider> new_provider);
    std::unordered_set<RobotIdentifier> getRobotsForRole(const std::string& role);
    std::unordered_set<std::string> getRoles() const;

   private:
    mutable std::mutex provider_mutex;
    logger::Logger logger{"role_manager"};
    game_data_provider::GameDataProvider& game_data_provider;
    marker::MarkerService& ms;
    std::optional<std::shared_ptr<RoleProvider>> role_provider{std::nullopt};
    std::unordered_map<std::string, std::unordered_set<RobotIdentifier>> assigned_roles;
    std::unordered_map<std::string, std::string> assigned_colors;
    constexpr static double ROLE_ASSIGN_RATE = 1.0;
    time::Rate rate{ROLE_ASSIGN_RATE};

    /**
     * @brief displays a torus with a color indicating the assigned role for every ally robot
     *
     */
    void displayRoleMarkers(const std::vector<std::pair<transform::RobotHandle, std::string>>& assigned_roles);
};

}  // namespace luhsoccer::role_manager