#pragma once

#include <optional>
#include <utility>

#include "transform/handles.hpp"
#include "observer/robot_datatypes.hpp"

namespace luhsoccer::observer::calculation {

/**
 * @brief Used to calculate the goal Probability of an ally robot
 *
 * @param ally_handle A RobotHandle to an ally
 * @param time The timepoint of the data which should be considered
 * @return std::optional<double> An Optional of the goal probability
 */
std::optional<double> calculateGoalProbability(const transform::RobotHandle& ally_handle,
                                               const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Used to calculate the BestPassReceiver for a given ally robot-handle
 * @deprecated Use calculatePassProbability to calculate the pass probability between two robots
 *
 * @param ally_handle A transform::RobotHandle to an ally Robot
 * @return std::optional<AllyRobot::BestPassReceiver> A BestPassReceiver object containing the best-pass-receiver
 * identifier and the score for the pass
 */
std::optional<AllyRobot::BestPassReceiver> calculateBestPassReceiver(
    const transform::RobotHandle& ally_handle, std::vector<RobotIdentifier> allies_to_evaluate = {},
    const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Calculates the pass probability between two robots
 *
 * @param passing_robot
 * @param receiving_robot
 * @param world_model_ptr
 * @param time
 * @return std::optional<double>
 */
std::optional<double> calculatePassProbability(const transform::RobotHandle& passing_robot,
                                               const RobotIdentifier& receiving_robot,
                                               const std::shared_ptr<const transform::WorldModel>& world_model_ptr,
                                               const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Used to calculate the current BallHolder and the movement permisseveness
 *
 * @note This function does not Update the LastBallObtainedPos in the WorldModel. Because of this the movement
 * permisseveness status is NOT guaranteed to be accurate
 *
 * @note This Function does not calculate which robot currently has the ball in its dribbler but which robot is roughly
 * in controll of the ball
 *
 * @param game_data_provider The gdp
 * @return std::optional<BallHolder> A BallHolder Object containing a RobotIdentifier to the current ball-holder, the
 * position where the robot obtained the ball and a bool stating whether the Robot is allowed to move
 */
std::optional<BallHolder> calculateBallPosession(const game_data_provider::GameDataProvider& gdp);

/**
 * @brief Used to calculate the Threat score of a given enemy Robot
 *
 * @param enemy_handle A RobotHandle to a Enemy Robot
 * @return std::optional<double> An optional of the threat score of the given enemy
 */
std::optional<double> calculateThreatLevel(const transform::RobotHandle& enemy_handle);

/**
 * @brief Evaluate whether the robot 'passer' has a clear pass line to the 'receiver'
 *
 * @param passer The Robot with the ball
 * @param receiver The Robot awaiting the Ball
 * @return std::optional<bool> Whether the line between the passer and the Receiver is covered
 */
std::optional<bool> isPassLineDefended(const transform::RobotHandle& passer, const transform::RobotHandle& receiver,
                                       const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Evaluate whether the robot 'ball_holder' has all outgoing pass-lines covered
 *
 * @param ball_holder The Robot which is evaluated (assume it is holding the ball)
 * @return std::optional<bool> Whether all outgoing pass opportunities are covered
 */
std::optional<bool> isOutgoingPassDefended(const transform::RobotHandle& ball_holder,
                                           const time::TimePoint time = time::TimePoint(0));

// @todo Work on correct physics / Approximation
/**
 * @brief Used to calculate the Position of the Ball after a specific time
 *
 * @param world_model A Reference to the World-Model
 * @param time_from_now The ammount of time from now how long the Ball should travel (in milliseconds)
 * @return std::optional<Eigen::Vector2d> The Position of the Ball *time_from_now* milliseconds from now
 */
std::optional<Eigen::Vector2d> calculateFutureBallPos(std::shared_ptr<const transform::WorldModel> world_model,
                                                      time::Duration time_from_now);

/**
 * @brief Used to calculate the Position inside with the biggest scoring propability
 *
 * @param robot A robot handle
 * @param id_to_ignore (optional) A robot which should be ignored in the calculation
 * @param time The tome at which the calculation should take place (in the past)
 * @return std::optional<Eigen::Vector2d> the Point with the biggest scoring propability or nullopt if no
 * good point was found
 */
std::optional<Eigen::Vector2d> calculateBestGoalPoint(const transform::RobotHandle& robot,
                                                      std::optional<RobotIdentifier> id_to_ignore = std::nullopt,
                                                      const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Used to calculate the point in the Ally Goal which is least defended for a given Enemy
 *
 * @param robot The Enemy
 * @param gdp The Game Data Provider
 * @param time The Time
 * @return std::optional<Eigen::Vector2d> The Point in our goal which is least protected from the enemy
 */
std::optional<Eigen::Vector2d> calculateMostLikelyShootPoint(const transform::RobotHandle& enemy,
                                                             const game_data_provider::GameDataProvider& gdp,
                                                             const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Calculates the point on the Wall the enemy would hit if it made a straight, uninterrupted shot
 *
 * @param robot A Handle to the robot
 * @param time The tome at which the calculation should take place (in the past)
 * @return An optional of a pair containing 1: The point the enemy would hit, 2: if this point is in OUR goal
 */
std ::optional<std::pair<Eigen::Vector2d, bool>> calculateShootPoint(const transform::RobotHandle& robot,
                                                                     const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Calculates whether the given robot is inside a specific angle (based on ball movement) to see if it is viable
 * for intercepting a Ball
 *
 * @param handle The given Robot
 * @param time The time
 * @return std::optional<double> Whether the robot is viable
 */
std::optional<bool> calculateInterceptionRobotIsViable(const transform::RobotHandle& handle,
                                                       const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Calculates the best possible interceptor for the ball
 *
 * @param world_model The World Model
 * @param time The time
 * @return std::optional<transform::RobotHandle> std::nullopt if the ball doesnt move enough or if no best interceptor
 * exists
 */
std::optional<transform::RobotHandle> calculateBestInterceptor(const transform::WorldModel& world_model,
                                                               std::vector<RobotIdentifier> viable_interceptors = {},
                                                               const time::TimePoint time = time::TimePoint(0));

}  // namespace luhsoccer::observer::calculation
