#pragma once

#include <Eigen/Geometry>
#include <optional>
#include <vector>
#include <functional>
#include <unordered_map>

#include "common_types.hpp"

#include "observer/robot_datatypes.hpp"

namespace luhsoccer::observer {

class DataStorage {
   public:  // Constructor
    DataStorage() : logger("observer"){};

   public:  // Getter Ally
    /**
     * @brief Get the best possible pass receiver for a given Ally-Identifier
     *
     * @param ally The Ally-Identifier
     * @return std::optional<AllyRobot::BestPassReceiver> An optional to the best-pass-receiver of the ally robot
     */
    [[nodiscard]] std::optional<AllyRobot::BestPassReceiver> getBestPassReceiver(const RobotIdentifier& ally) const;

    /**
     * @brief Get the goal probability of a certain ally Robot
     *
     * @param ally The RobotHandle to an ally robot
     * @return std::optional<double> The goal probability in percent(%) (0.0 - 100.0)
     */
    [[nodiscard]] std::optional<double> getGoalProbability(const RobotIdentifier& ally) const;

   public:  // Getter Enemy
    /**
     * @brief Get the Threat-Level of a enemy Robot
     *
     * @param enemy The handle to the enemy Robot from which we want the Threat-Level
     * @return std::optional<double> The Threat-Level of the given enemy Robot (only if the Robot was found)
     */
    [[nodiscard]] std::optional<double> getThreatLevel(const RobotIdentifier& enemy) const;

   public:  // Getter General
    /**
     * @brief Get the Ball Goal Probability
     *
     * @return double The ball goal probability in percent (0.0 - 100.0)
     */
    [[nodiscard]] double getBallGoalProbability() const;

    /**
     * @brief Get the Ball Holder
     *
     * @return std::optional<RobotHandle> RobotHandle if there is a ballholder, std::nullopt if there isnt
     */
    [[nodiscard]] std::optional<transform::RobotHandle> getBallCarrier() const;

    /**
     * @brief Get the Ball Controlling Team
     *
     * @return std::optional<Team> Team enum if a team controlls the ball, std::nullopt otherwise
     */
    [[nodiscard]] std::optional<Team> getBallControllingTeam() const;

    /**
     * @brief Get the Position where the Robot which is currently holding the ball has obtained it
     *
     * @return std::optional<transform::Position>
     */
    [[nodiscard]] std::optional<Eigen::Vector2d> getBallObtainedPos() const;

    /**
     * @brief Get the current Strategy
     *
     * @return StrategyType The current Strategy (Offensive or Defensive)
     */
    [[nodiscard]] StrategyType getStrategyType() const;

    /**
     * @brief Get the Last Ball Toucher
     *
     * @return std::optional<transform::RobotHandle> The last ball toucher
     */
    [[nodiscard]] std::optional<transform::RobotHandle> getLastBallToucher() const;

    /**
     * @brief Get the Robot who is not allowed to touch the Ball
     *
     * @return std::optional<transform::RobotHandle> The Robot
     */
    [[nodiscard]] std::optional<transform::RobotHandle> getBallTouchingForbiddenRobot() const;

   public:  // General Methods
    /**
     * @brief Stores an empty Robot object for the given handle
     *
     * @param handle The handle for the Robot
     */
    void addEmptyRobot(const RobotIdentifier& handle);

   public:  // Setter for Ally Robots
    /**
     * @brief Set the Best Pass Receiver score and handle for an AllyRobot
     *
     * @param handle The handle to the Robot for which the calculation was performed
     * @param bpr_handle The handle to the Robot which is the Best Pass Receiver
     * @param score The pass chance(defaults to 0.0 if the BestPassReceiver is a std::nullopt)
     */
    void setBestPassReceiver(const RobotIdentifier& handle, transform::RobotHandle bpr_handle, double score = 0.0);

    /**
     * @brief Set the probability that an ally Robot will score a goal from its current position
     *
     * @param handle The handle to the Ally Robot
     * @param score_with_goalie The real goal probability
     * @param score_without_goalie The goal probability neglecting the goalie
     */
    void setGoalProbability(const RobotIdentifier& handle, double score);

   public:  // Setter for Enemy Robots
    /**
     * @brief Set the threat Level of an enemy Robot
     *
     * @param handle The handle to the enemy Robot
     * @param threat_score The new threat level (See ThreadLevel Enum)
     */
    void setThreatLevel(const RobotIdentifier& handle, double threat_score);

    /**
     * @brief Set the pass Defended state of an enemy Robot
     *
     * @param handle The handle to the Enemy Robot
     * @param is_pass_defended Whether the enemy is currently pass defended
     */
    void setPassDefended(const RobotIdentifier& handle, bool is_pass_defended = false);

    /**
     * @brief Set the goal Defended state of an enemy Robot
     *
     * @param handle The handle to the Enemy Robot
     * @param is_goal_defended Whether the enemy is currently Goal defended
     */
    void setGoalDefended(const RobotIdentifier& handle, bool is_goal_defended = false);

    /**
     * @brief Set the defense-area-defended state of an enemy Robot
     *
     * @param handle The handle to the Enemy Robot
     * @param is_defense_area_defended Whether the enemy is currently defende-area-defended
     */
    void setDefenseAreaDefended(const RobotIdentifier& handle, bool is_defense_area_defended = false);

   public:  // Setter for single Objects
    /**
     * @brief Set the Goal Probability for the Ball
     *
     * @param new_val The new Goal Probability for the Ball
     */
    void setBallGoalProbability(double new_val);

    /**
     * @brief Sets the new Ball Holder & sets the carries_ball attribute of all other Robots to false
     *
     * @param new_robot a Handle to the new Ball Holder (std::nullopt if noone holds the ball)
     */
    void setBallHolder(std::optional<BallHolder> new_robot);

    /**
     * @brief Set the new Strategy-Type
     *
     * @param strategy_type The new Strategy Type (Defensive or Offensive)
     */
    void setStrategyType(StrategyType strategy_type);

    /**
     * @brief Set the Last Ball Toucher
     *
     * @param handle The last ball toucher
     */
    void setLastBallToucher(std::optional<transform::RobotHandle> handle);

    /**
     * @brief Set the Robot which is not allowed to touch the Ball
     *
     * @param handle The Robot
     */
    void setBallTouchingForbiddenRobot(std::optional<transform::RobotHandle> handle);

   private:  // Variables / Data
    /**
     * @brief A Map used to store Data about ally Robots
     *
     */
    std::unordered_map<RobotIdentifier, AllyRobot> ally_robots;

    /**
     * @brief A Map used to store Data about enemy Robots
     *
     */
    std::unordered_map<RobotIdentifier, EnemyRobot> enemy_robots;

    /**
     * @brief Stores Info about the current BallHolder (RobotIdentifier, movement_allowed, BallObtainedPos)
     *
     */
    std::optional<BallHolder> ball_holder = std::nullopt;

    /**
     * @brief The current goal probability from the perspective of the Ball
     *
     */
    double ball_goal_probability = 0.0;

    /**
     * @brief The current strategy Type (Offensive / Defensive)
     *
     */
    StrategyType current_behaviour = StrategyType::DEFENSIVE;

    /**
     * @brief The Robot who last touched the Ball
     */
    std::optional<transform::RobotHandle> last_ball_toucher = std::nullopt;

    /**
     * @brief The Robot who is not allowed to touch the ball currently
     *
     */
    std::optional<transform::RobotHandle> ball_touch_forbidden_robot = std::nullopt;

   private:  // Target Points
    struct {
        // Store Transform positions / or own positions? as std::optionals
    } points;

    logger::Logger logger;
};

}  // namespace luhsoccer::observer
