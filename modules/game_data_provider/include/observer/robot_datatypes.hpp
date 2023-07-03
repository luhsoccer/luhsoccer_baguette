#pragma once

#include <optional>
#include <Eigen/Geometry>
#include <utility>
#include "common_types.hpp"
#include "transform/handles.hpp"

namespace luhsoccer::observer {

enum class Gamemode {
    DEFENDING,
    NEUTRAL,
    ATTACKING,
};

enum class StrategyType { OFFENSIVE, DEFENSIVE };

struct RobotBase {};

/**
 * @brief A struct used to store all kinds of info relevant for an ally Robot
 */
struct AllyRobot : public RobotBase {
    AllyRobot() = default;
    AllyRobot(const RobotBase& base) : RobotBase(base) {}

    /**
     * @brief The BestPassReceiver stores a RobotHandle to the best suited ally robot for a pass and the associated
     * score
     */
    struct BestPassReceiver {
        BestPassReceiver(transform::RobotHandle handle, double score) : handle(std::move(handle)), score(score) {}
        transform::RobotHandle handle;
        double score = 0.0;
    };

    std::optional<BestPassReceiver> best_pass_receiver = std::nullopt;

    /**
     * @brief The goal_probability is the probability (0.0 - 100.0) which states how likely the Robot is to score a goal
     * from the current GameState
     */
    double goal_probability = 0.0;
};

/**
 * @brief A struct used to store Info about an enemy Robot
 *
 */
struct EnemyRobot : public RobotBase {
    EnemyRobot() = default;
    EnemyRobot(const RobotBase& base) : RobotBase(base) {}

    double threat_score = 0;

    bool pass_defended = false;

    bool goal_defended = false;

    bool defense_area_defended = false;
};

/**
 * @brief Stores the handle to the Robot who controlls the ball & the position where the Robot OBTAINED the ball
 *
 */
struct BallHolder {
    BallHolder(transform::RobotHandle handle, std::optional<Eigen::Vector2d> pos, bool movement_allowed = true)
        : handle(std::move(handle)), ball_obtained_pos(std::move(pos)), movement_allowed(movement_allowed) {}

    transform::RobotHandle handle;

    std::optional<Eigen::Vector2d> ball_obtained_pos;

    bool movement_allowed = true;
};

}  // namespace luhsoccer::observer