#pragma once

#include <optional>
#include <utility>

#include "transform/handles.hpp"
#include "observer/robot_datatypes.hpp"
#include "utils/luts.hpp"

namespace luhsoccer::observer::calculation {

/**
 * @brief Used to calculate the goal Probability of the ball
 *
 * @param ally_handle A Position, a Team and a Worldmodel
 * @return std::optional<double> An Optional of the goal probability from the ball
 */
std::optional<double> calculateGoalProbability(const transform::Position& pos, Team team,
                                               std::shared_ptr<const transform::WorldModel> wm,
                                               const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Used to calculate the Position inside with the biggest scoring propability
 *
 * @param robot A position of a Robot, a Team and a Worldmodel
 * @return std::optional<Eigen::Vector2d> The Point with the biggest scoring propability
 */
std::optional<Eigen::Vector2d> calculateBestGoalPoint(const transform::Position& pos, Team team,
                                                      std::shared_ptr<const transform::WorldModel> wm,
                                                      std::optional<RobotIdentifier> id_to_ignore = std::nullopt,
                                                      const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Used to calculate the Threat score of a given position
 *
 * @param enemy_pos The position of the robot
 * @param world_model A shared_ptr to the world-model
 * @param time The time point
 * @return std::optional<double> The calculated ThreatScore
 */
std::optional<double> calculateThreatLevel(const transform::Position& enemy_pos,
                                           std::shared_ptr<const transform::WorldModel> world_model,
                                           const time::TimePoint time = time::TimePoint(0));

std::optional<bool> isPassLineDefended(const transform::Position& passer, const transform::Position& receiver,
                                       const Team team, std::shared_ptr<const transform::WorldModel> world_model,
                                       const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Evaluate whether the robot 'ball_holder' has all outgoing pass-lines covered
 *
 * @param robot The transform::Position Robot which is evaluated (assume it is holding the ball)
 * @param team The Team of the Robot wich is evaluated
 * @param world_model A shared pointer of a worldmodel
 * @param robot_id The id of the robot whose position was give (only if that robot should be ignored)
 * @param time time point at wich the function should be evaluated
 * @return std::optional<bool> Whether all outgoing pass opportunities are covered
 */
std::optional<bool> isOutgoingPassDefended(const transform::Position& robot, Team team,
                                           std::shared_ptr<const transform::WorldModel> world_model,
                                           std::optional<RobotIdentifier> robot_id = std::nullopt,
                                           const time::TimePoint time = time::TimePoint(0));

/**
 * @brief
 * @deprecated The best pass receiver should be calculated by yourself (use the method )
 *
 * @param passer
 * @param team
 * @param world_model
 * @param allies_to_evaluate
 * @param passing_robot
 * @param time
 * @return std::optional<AllyRobot::BestPassReceiver>
 */
std::optional<AllyRobot::BestPassReceiver> calculateBestPassReceiver(
    const transform::Position& passer, Team team, std::shared_ptr<const transform::WorldModel> world_model,
    std::vector<RobotIdentifier> allies_to_evaluate = {}, std::optional<RobotIdentifier> passing_robot = std::nullopt,
    const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Calculates the pass probability between robots which stand an the two given positions
 *
 * @param passing_robot_pos The Position of the Robot which is performing the pass
 * @param receiving_robot_pos The Position of the Robot which is accepting the pass
 * @param team The team of the two robots
 * @param world_model_ptr A pointer to the world model
 * @return std::optional<double> The pass probability
 */
std::optional<double> calculatePassProbability(const transform::Position& passing_robot_pos,
                                               const transform::Position& receiving_robot_pos, const Team team,
                                               const std::shared_ptr<const transform::WorldModel>& world_model_ptr,
                                               const time::TimePoint time = time::TimePoint(0));

std::optional<std::pair<Eigen::Vector2d, bool>> calculateShootPoint(
    const transform::Position& robot, Team team, std::shared_ptr<const transform::WorldModel> world_model,
    const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Copied from python for performance reasons
 *
 * @param start_pos
 * @param end_pos
 * @param kicker_pos
 * @param enemies
 * @param time
 * @param pass_power
 * @param enemy_max_speed
 * @param radius_scaling
 * @param pass_power_lut
 * @param turn_time_lut
 * @param pass_time_lut
 * @return double
 */
double calcPassSuccessChance(const Eigen::Vector2d& start_pos, const Eigen::Vector2d& end_pos,
                             const Eigen::Vector2d& kicker_pos, const std::vector<Eigen::Vector2d>& enemies,
                             double time, double pass_power, double enemy_max_speed, double radius_scaling,
                             const util::Lut1D& pass_power_lut, const util::Lut1D& turn_time_lut,
                             const util::Lut2D& pass_time_lut);

}  // namespace luhsoccer::observer::calculation
