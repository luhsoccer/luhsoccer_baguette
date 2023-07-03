#pragma once

#include "transform/world_model.hpp"

#include "observer/buffer.hpp"
#include "observer/data_storage.hpp"

namespace luhsoccer::config_provider {
struct ConfigStore;
}

namespace luhsoccer::observer {

class Observer {
   public:  // Main Methods
    Observer(std::weak_ptr<const transform::WorldModel> world_model);
    Observer();

    /**
     * @brief Set the WorldModel for this Observer
     *
     * @param wm A weak_ptr to a WorldModel
     */
    void setWorldModel(std::weak_ptr<const transform::WorldModel> wm);

    /**
     * @brief Update Method of the Observer should be called for every calculation
     *
     * @param gdp The gdp
     */
    void update(const game_data_provider::GameDataProvider& gdp);

    /* ------------------------- Getter Methods ------------------------- */
   public:  // Getter Ally
    /**
     * @brief Get the best pass Receiver for a given ally Robot
     *
     * @param ally A Identifier of a ally robot
     * @return std::optional<AllyRobot::BestPassReceiver> An Optional of the BestPassReceiver
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
     * @return std::optional<ThreatLevel> The Threat-Level of the given enemy Robot (only if the Robot was found)
     */
    [[nodiscard]] std::optional<double> getThreatLevel(const RobotIdentifier& enemy) const;

   public:  // Getter General
    /**
     * @brief Get the Allowed Dribble Distance for a given robot
     *
     * @return std::optional<double> The allowed travel distance (nullopt if unlimited)
     */
    [[nodiscard]] std::optional<double> getAllowedDribbleDistance(const RobotIdentifier& id) const;

    /**
     * @brief Get the Ball Goal Probability
     *
     * @return double The ball goal probability in percent (0.0 - 100.0)
     */
    [[nodiscard]] double getBallGoalProbability() const;

    /**
     * @brief Get the Ball Holder
     *
     * @return std::optional<RobotId> RobotId if there is a ballholder, std::nullopt if there isnt
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
     * @brief Get the robot who last toucched the ball
     *
     * @return std::optional<transform::RobotHandle> The robot who last touched the ball
     */
    [[nodiscard]] std::optional<transform::RobotHandle> getLastBallTouchingRobot() const;

    /**
     * @brief Get the Robot which is not allowed to touch the Ball
     *
     * @return std::optional<transform::RobotHandle> The Robot
     */
    [[nodiscard]] std::optional<transform::RobotHandle> getBallTouchingForbiddenRobot() const;

    /* ------------------------- Calculation Methods ------------------------- */
   private:  // Update Methods
    /**
     * @brief Updates the best_pass_receiver for every ally robot (handle and score)
     *
     */
    void updateBestPassReceiver(const std::shared_ptr<const transform::WorldModel>& world_model_shared);

    /**
     * @brief Updates the goal probability for every ally robot and the ball_goal_probability score
     *
     */
    void updateGoalProbability(const std::shared_ptr<const transform::WorldModel>& world_model_shared);

    /**
     * @brief Updates the ball_holder, the carries_ball & movement_allowed attribute of the ball-carrying robot
     *
     * !!! GDP has to set the ball-obtained-pos in the WM after every update call; remove it if the Observer returns a
     * nullopt for the ball-obtained-pos !!!
     */
    void updateBallPosession(const game_data_provider::GameDataProvider& gdp);

    /**
     * @brief Updates the Threat level of every Robot
     *
     */
    void updateEnemyThreatLevel(const std::shared_ptr<const transform::WorldModel>& world_model_shared);

    /**
     * @brief Updates whether the current-playstyle should be offensive or defensive (current_behaviour value)
     *
     */
    void updateStrategyType(const std::shared_ptr<const transform::WorldModel>& world_model_shared);

   private:  // Variables
    /**
     * @brief A weak-pointer to the World-Model this Observer instance works on
     *
     */
    std::weak_ptr<const transform::WorldModel> world_model;

    /**
     * @brief An Object which stores the calculated data in a front-back-buffer style
     *
     */
    buffer::BackFrontBuffer<DataStorage> data_storage_buffer;

    /**
     * @brief The current score used to calculate the strategy type
     */
    time::Duration ball_posession_timer = 0.0;

    std::optional<transform::RobotHandle> double_touch_last_ball_holder = std::nullopt;

    config_provider::ConfigStore& cs;

    logger::Logger logger;
};

}  // namespace luhsoccer::observer