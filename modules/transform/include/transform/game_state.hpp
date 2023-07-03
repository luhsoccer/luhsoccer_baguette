#pragma once

#include <string>

namespace luhsoccer::transform {

/// @brief State of the game
enum class GameState {
    /**
     * @brief no movement, no ball manipulation
     *
     */
    HALT,
    /**
     * @brief 1.5m/s max speed, 0.5m distance to ball
     *
     */
    STOP,
    /**
     * @brief normal play
     *
     */
    NORMAL,
    /**
     * @brief our team needs to place the ball, our robots need to be 0.05m away
     *
     */
    BALL_PLACEMENT_FREE_KICK,
    /**
     * @brief our team needs to place the ball, our robots need to be 0.5m away
     *
     */
    BALL_PLACEMENT_FORCE_START,
    /**
     * @brief the other team places the ball
     *
     */
    BALL_PLACEMENT_ENEMY,
    /**
     * @brief our team will get kickoff
     *
     */
    KICKOFF_PREP,
    /**
     * @brief the other team will get a kickoff
     *
     */
    KICKOFF_PREP_ENEMY,
    /**
     * @brief our team executes the kickoff
     *
     */
    KICKOFF,
    /**
     * @brief the other team executes the kickoff
     *
     */
    KICKOFF_ENEMY,
    /**
     * @brief our team will get a free kick
     *
     */
    FREE_KICK_PREP,
    /**
     * @brief the other team will get a free kick
     *
     */
    FREE_KICK_PREP_ENEMY,
    /**
     * @brief our team executes the free kick
     *
     */
    FREE_KICK,
    /**
     * @brief the other team executes the free kick
     *
     */
    FREE_KICK_ENEMY,
    /**
     * @brief our team will get a penalty kick
     *
     */
    PENALTY_PREP,
    /**
     * @brief the other team will get a penalty kick
     *
     */
    PENALTY_PREP_ENEMY,
    /**
     * @brief our team executes the penalty kick
     *
     */
    PENALTY,
    /**
     * @brief the other team executes the penalty
     *
     */
    PENALTY_ENEMY,

    /**
    * @brief our team is in a timeout
    *
    */
    TIMEOUT_ALLY,

    /**
    * @brief enemy team is in a timeout
    *
    */

    TIMEOUT_ENEMY
};

/**
 * @brief Get the name of the game state
 *
 * @param state the state to get the name of
 * @return std::string_view
 */
inline std::string_view getGameStateName(const GameState& state) {
    switch (state) {
        case GameState::HALT:
            return "HALT";
        case GameState::STOP:
            return "STOP";
        case GameState::NORMAL:
            return "NORMAL";
        case GameState::BALL_PLACEMENT_FREE_KICK:
            return "BALL_PLACEMENT_FREE_KICK";
        case GameState::BALL_PLACEMENT_FORCE_START:
            return "BALL_PLACEMENT_FORCE_START";
        case GameState::BALL_PLACEMENT_ENEMY:
            return "BALL_PLACEMENT_ENEMY";
        case GameState::KICKOFF_PREP:
            return "KICKOFF_PREP";
        case GameState::KICKOFF_PREP_ENEMY:
            return "KICKOFF_PREP_ENEMY";
        case GameState::KICKOFF:
            return "KICKOFF";
        case GameState::KICKOFF_ENEMY:
            return "KICKOFF_ENEMY";
        case GameState::FREE_KICK_PREP:
            return "FREE_KICK_PREP";
        case GameState::FREE_KICK_PREP_ENEMY:
            return "FREE_KICK_PREP_ENEMY";
        case GameState::FREE_KICK:
            return "FREE_KICK";
        case GameState::FREE_KICK_ENEMY:
            return "FREE_KICK_ENEMY";
        case GameState::PENALTY_PREP:
            return "PENALTY_PREP";
        case GameState::PENALTY_PREP_ENEMY:
            return "PENALTY_PREP_ENEMY";
        case GameState::PENALTY:
            return "PENALTY";
        case GameState::PENALTY_ENEMY:
            return "PENALTY_ENEMY";
        case GameState::TIMEOUT_ENEMY:
            return "TIMEOUT_ENEMY";
        case GameState::TIMEOUT_ALLY:
            return "TIMEOUT_ALLY";

            break;
    }
    return "UNKOWN";
}

inline std::ostream& operator<<(std::ostream& os, const GameState& state) { return os << getGameStateName(state); }

}  // namespace luhsoccer::transform