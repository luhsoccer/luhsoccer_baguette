#pragma once

#include <optional>
#include <Eigen/Geometry>
#include <utility>
#include "transform/handles.hpp"

namespace luhsoccer::transform::helper {

/**
 * @brief Used to get the absolute Position of a Robot in respect to the Global Frame
 *
 * @param handle A RobotHandle to the Robot for which the Position is requested
 * @return std::optional<Eigen::Vector2d> The Position of the Robot in the Global Frame (x, y)
 */
[[nodiscard]] std::optional<Eigen::Vector2d> getPosition(const transform::RobotHandle& handle,
                                                         const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Used to get the position and rotation of a Robot in respect to the Global Frame
 *
 * @param handle A RobotHandle to the Robot for which the Position and Rotation is requested
 * @return std::optional<Eigen::Vector3d> A Pair of the 2d position of the Robot and the rotation (x, y, z)
 */
[[nodiscard]] std::optional<Eigen::Vector3d> getPositionAndRotation(const transform::RobotHandle& handle,
                                                                    const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Used to get the Velocity and rotational Velocity of a Robot in respect to the Global Frame
 *
 * @param handle A RobotHandle to the Robot for which the Velocity and rotational Velocity is requested
 * @return std::optional<Eigen::Vector3d> A Vector containing x, y and theta velocity (x, y, theta)
 */
[[nodiscard]] std::optional<Eigen::Vector3d> getVelocity(const transform::RobotHandle&,
                                                         const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Used to get the Position of the Ball in respect to the Global Frame
 *
 * @param wm A World Model
 * @return std::optional<Eigen::Vector2d> The Position of the Ball
 */
[[nodiscard]] std::optional<Eigen::Vector2d> getBallPosition(const transform::WorldModel& wm,
                                                             const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Used to get the Velocity of the Ball in respect to the Global Frame
 *
 * @param wm A World Model
 * @return std::optional<Eigen::Vector3d> The Velocity of the Ball
 */
[[nodiscard]] std::optional<Eigen::Vector3d> getBallVelocity(const transform::WorldModel& wm,
                                                             const time::TimePoint time = time::TimePoint(0));

}  // namespace luhsoccer::transform::helper
