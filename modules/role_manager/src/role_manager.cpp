#include "role_manager/role_manager.hpp"
#include "game_data_provider/game_data_provider.hpp"
#include "observer/continuous_observer.hpp"
#include "transform/handles.hpp"
#include "marker_service/marker_service.hpp"
#include <algorithm>

namespace luhsoccer::role_manager {

namespace {
std::optional<marker::Color> parseColor(std::string& color) {
    unsigned char r = std::strtol(color.substr(0, 2).c_str(), nullptr, 16);
    unsigned char g = std::strtol(color.substr(2, 2).c_str(), nullptr, 16);
    unsigned char b = std::strtol(color.substr(4, 2).c_str(), nullptr, 16);

    return marker::Color{static_cast<double>(r), static_cast<double>(g), static_cast<double>(b)};
}

std::optional<std::string> findAndExtractColor(std::string& role) {
    // Check if there is any # in the string
    auto iter = role.find_first_of('#');
    if (iter == std::string::npos) {
        return std::nullopt;
    }

    if (iter + 6 >= role.length()) {
        role.erase(iter, role.length());
        return std::nullopt;
    }

    auto color_part = role.substr(iter + 1);

    role.erase(iter, role.length());

    return color_part;
}
}  // namespace

RoleManager::RoleManager(game_data_provider::GameDataProvider& provider, marker::MarkerService& ms)
    : game_data_provider(provider), ms(ms) {}

void RoleManager::setRoleProvider(std::shared_ptr<RoleProvider> new_provider) {
    std::lock_guard lock(this->provider_mutex);
    if (new_provider != nullptr) {
        this->role_provider = std::move(new_provider);
    } else {
        this->role_provider = std::nullopt;
    }
}

void RoleManager::displayRoleMarkers(
    const std::vector<std::pair<transform::RobotHandle, std::string>>& assigned_roles) {
    constexpr size_t RED_SHIFT = 16;
    constexpr size_t GREEN_SHIFT = 8;
    constexpr int PART_RED = 0xFF0000;
    constexpr int PART_GREEN = 0x00FF00;
    constexpr int PART_BLUE = 0x0000FF;
    constexpr double TORUS_SCALE = 1.2;
    size_t i = 0;
    // loop through all assigned roles and display a 3d circle at the robots position
    for (const auto& [robot, role] : assigned_roles) {
        size_t hash = std::hash<std::string>{}(role);
        size_t red = (hash & PART_RED) >> RED_SHIFT;
        size_t green = (hash & PART_GREEN) >> GREEN_SHIFT;
        size_t blue = hash & PART_BLUE;
        marker::Torus torus{robot.getPosition(), "roles", i};

        bool assign_default_color = false;

        if (this->assigned_colors.contains(role)) {
            auto color = parseColor(this->assigned_colors[role]);
            if (color) {
                torus.setColor(color.value());
            } else {
                assign_default_color = true;
            }
        } else {
            assign_default_color = true;
        }

        if (assign_default_color) {
            torus.setColor(
                marker::Color{static_cast<double>(red), static_cast<double>(green), static_cast<double>(blue)});
        }

        torus.setScale(TORUS_SCALE);
        torus.setLifetime(2);  // TODO change lifetime as needed
        ms.displayMarker(torus);
        ++i;
    }
}

void RoleManager::loop(std::atomic_bool& /*should_run*/) {
    if (this->role_provider) {
        auto visible_allies = game_data_provider.getWorldModel()->getVisibleRobots<Team::ALLY>();
        auto visible_enemies = game_data_provider.getWorldModel()->getVisibleRobots<Team::ENEMY>();

        std::vector<transform::RobotHandle> allies;
        std::vector<transform::RobotHandle> enemies;
        allies.reserve(visible_allies.size());
        enemies.reserve(visible_enemies.size());

        for (const auto& id : visible_allies) {
            allies.emplace_back(id, game_data_provider.getWorldModel());
        }

        for (const auto& id : visible_enemies) {
            enemies.emplace_back(id, game_data_provider.getWorldModel());
        }

        // @todo for some reason this does not work??
        // std::transform(
        //     visible_ids.cbegin(), visible_ids.cend(), visible_robots.begin(),
        //     [&](const RobotIdentifier& id) { return transform::RobotHandle(id, game_data_provider.getWorldModel());
        //     });

        auto possible_roles = this->role_provider->get()->getPossibleRoles();

        const auto assigned_roles = this->role_provider->get()->provideRoles(std::move(allies), std::move(enemies),
                                                                             game_data_provider.getObserver());

        std::unique_lock lock(this->provider_mutex);
        this->assigned_roles.clear();
        this->assigned_colors.clear();

        for (auto& role : possible_roles) {
            const auto color = findAndExtractColor(role);
            if (color) {
                this->assigned_colors[role] = color.value();
            }
        }

        for (const auto& [id, role] : assigned_roles) {
            this->assigned_roles[role].insert(id.getID());
        }

        lock.unlock();

        // All roles match
        // LOG_INFO(logger, "Assigned roles: {}", assigned_roles);
        // @todo luhviz
        displayRoleMarkers(assigned_roles);

        // @todo next step
    } else {
        // LOG_WARNING(logger, "No role provider given");
    }

    this->rate.sleep();
}

std::unordered_set<RobotIdentifier> RoleManager::getRobotsForRole(const std::string& role) {
    std::lock_guard lock(this->provider_mutex);
    return this->assigned_roles[role];
}

std::unordered_set<std::string> RoleManager::getRoles() const {
    std::lock_guard lock(this->provider_mutex);
    std::unordered_set<std::string> keys;
    std::transform(this->assigned_roles.begin(), this->assigned_roles.end(), std::inserter(keys, keys.end()),
                   [](auto pair) { return pair.first; });
    return keys;
}

}  // namespace luhsoccer::role_manager