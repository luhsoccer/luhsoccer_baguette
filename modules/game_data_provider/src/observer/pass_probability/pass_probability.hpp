#pragma once

#include "transform/handles.hpp"
#include "robot_identifier.hpp"
#include "utils/luts.hpp"

namespace luhsoccer::observer::calculation::pass_probability {

/**
 * @brief Calculates the pass probability between the two given handles
 *
 * @param passing_robot The robot which is supposed to perform the pass
 * @param receiving_robot The robot which is supposed to accept the pass
 * @param world_model_ptr A ptr to the World model
 * @return std::optional<double> The pass probability
 */
std::optional<double> calculatePassProbability(const transform::RobotHandle& passing_robot,
                                               const RobotIdentifier& receiving_robot,
                                               const std::shared_ptr<const transform::WorldModel>& world_model_ptr,
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

/**
 * @brief Calculates the two points on the circle around 'interceptor' that intercept with the line from 'robot1' to
 * 'robot2'
 *
 * @param robot1
 * @param robot2
 * @param interceptor
 * @param interceptor_radius
 * @return std::optional<Eigen::Vector2d>
 */
std::optional<std::pair<Eigen::Vector2d, Eigen::Vector2d>> calculateCirclePoints(const Eigen::Vector2d& robot1,
                                                                                 const Eigen::Vector2d& robot2,
                                                                                 const Eigen::Vector2d& interceptor,
                                                                                 const double interceptor_radius);

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

}  // namespace luhsoccer::observer::calculation::pass_probability