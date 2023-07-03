#pragma once

#include <Eigen/Geometry>
#include "transform_helper/world_model_helper.hpp"

namespace luhsoccer::observer::utility {

/**
 * @brief Used to calculate the distance between two points represented as vectors
 *
 * @param v1 The first Point (Vector)
 * @param v2 The second Point (Vector)
 * @return double The distance between the two points
 */
inline double calculateDistance(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) { return (v1 - v2).norm(); }

/**
 * @brief Used to calculate the angle between two vectors in RADIANS in the range of [0; 2PI]
 *
 * @param v1 The first Vector
 * @param v2 The second Vector
 * @return double The angle between the two vectors
 */
inline double calculateAngle(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
    // The dot product of the two vectors
    const auto dot = v1.dot(v2);

    // The determinant of the Matrix resulting from appending the two vectors
    const double det = v1.x() * v2.y() - v1.y() * v2.x();

    // The angle between the two vectors
    const auto angle = atan2(det, dot);

    // correct the Angle (if it was negative add 2*PI)
    return angle + 2 * L_PI * (angle < 0);
}

/**
 * @brief Calculates a normalized vector which points in the direction the robot is facing
 *
 * @param rotation The angle of the Robot
 * @return Eigen::Vector2d A normalized vector containing the direction the robot is facing
 */
inline Eigen::Vector2d calculateRotationVector(double rotation) {
    return Eigen::Vector2d{cos(rotation), sin(rotation)};
}

// @todo Replace
inline std::optional<double> distanceToEnemyGoalPoint(const transform::RobotHandle& robot,
                                                      const time::TimePoint time = time::TimePoint(0)) {
    if (robot.getWorldModel().expired()) return std::nullopt;
    auto wm = robot.getWorldModel().lock();
    const auto pos = robot.getPosition().getCurrentPosition(wm, transform::field::GOAL_ENEMY_CENTER, time);
    if (!pos.has_value()) return std::nullopt;
    return pos->translation().norm();
}

// @todo Replace
inline std::optional<double> distanceToAllyGoalPoint(const transform::RobotHandle& robot,
                                                     const time::TimePoint time = time::TimePoint(0)) {
    if (robot.getWorldModel().expired()) return std::nullopt;
    auto wm = robot.getWorldModel().lock();
    const auto pos = robot.getPosition().getCurrentPosition(wm, transform::field::GOAL_ALLY_CENTER, time);
    if (!pos.has_value()) return std::nullopt;
    return pos->translation().norm();
}

}  // namespace luhsoccer::observer::utility