#pragma once

#include <string_view>

namespace luhsoccer {

constexpr inline double L_PI = 3.14159265358979323846;

constexpr inline size_t MAX_ROBOTS_PER_TEAM = 16;

enum class Team { ALLY, ENEMY };
enum class TeamColor { BLUE, YELLOW };
enum class Division { A, B };

constexpr inline Team getOppositeTeam(Team team) { return team == Team::ALLY ? Team::ENEMY : Team::ALLY; }

constexpr inline TeamColor getOppositeTeamColor(TeamColor color) {
    return color == TeamColor::BLUE ? TeamColor::YELLOW : TeamColor::BLUE;
}

constexpr inline Team operator!(const Team& rhs) { return getOppositeTeam(rhs); }
constexpr inline TeamColor operator!(const TeamColor& rhs) { return getOppositeTeamColor(rhs); }

std::string_view format_as(const Team& team);

std::string_view format_as(const TeamColor& team);

}  // namespace luhsoccer
