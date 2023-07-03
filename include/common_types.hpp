/**
 * @file common_types.hpp
 * @author Fabrice Zeug (zeug@stud.uni-hannover.de)
 * @brief Implements common data types for interfaces etc. important for all modules
 * @version 0.1
 * @date 2022-08-11
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <string>
#include <sstream>

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

inline std::ostream& operator<<(std::ostream& os, const Team& team) {
    switch (team) {
        case Team::ALLY:
            os << "Ally";
            break;
        case Team::ENEMY:
            os << "Enemy";
            break;
    }
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const TeamColor& team) {
    switch (team) {
        case TeamColor::BLUE:
            os << "Blue";
            break;
        case TeamColor::YELLOW:
            os << "Yellow";
            break;
    }
    return os;
}

}  // namespace luhsoccer
