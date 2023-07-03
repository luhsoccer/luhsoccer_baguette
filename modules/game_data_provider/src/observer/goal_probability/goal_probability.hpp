#include "transform/handles.hpp"

namespace luhsoccer::observer::calculation::goal_probability {

std::optional<std::vector<Eigen::Vector2d>> calculateShadows(const transform::Position& ball_position, Team team,
                                                             std::shared_ptr<const transform::WorldModel> wm,
                                                             std::optional<RobotIdentifier> id_to_ignore = std::nullopt,
                                                             const time::TimePoint time = time::TimePoint(0));

std::vector<Eigen::Vector2d> cutShadows(const std::vector<Eigen::Vector2d>& points);

std::vector<Eigen::Vector2d> combineShadows(std::vector<Eigen::Vector2d> shadows);

std::vector<Eigen::Vector2d> invertShadows(const std::vector<Eigen::Vector2d>& points);

[[nodiscard]] std::optional<double> distanceToRobotsTargetGoal(const transform::RobotHandle& handle,
                                                               const time::TimePoint time = time::TimePoint(0));

std::optional<double> goalFromDistanceProb(const transform::Position& ball_position, Team team,
                                           std::shared_ptr<const transform::WorldModel> wm,
                                           const time::TimePoint time = time::TimePoint(0));

/**
 * @brief Calculates a score based on what angle a robot has to its goal
 * When a robot is facing a goal at a steep angle the goal chance decreases
 *
 * @param ball_position The (assumed) position of the ball
 * @param team The Team which should score the goal
 * @param wm The World Model
 * @param time The time
 * @return std::optional<double> The Score which results from the angle of the robot to the goal
 */
std::optional<double> goalFromAngleProb(const transform::Position& ball_position, Team team,
                                        std::shared_ptr<const transform::WorldModel> wm,
                                        const time::TimePoint time = time::TimePoint(0));
}  // namespace luhsoccer::observer::calculation::goal_probability