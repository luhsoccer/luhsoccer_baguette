
enum class Team;
namespace luhsoccer::transform {
class WorldModel;
}

namespace luhsoccer::observer::calculation::misc {

/**
 * @brief A Function used to look whether a robot is inside a angle that is calculated based on given params
 * (Used for Ball-Interceptor)
 *
 * @param ball_robot_vec A vector from the ball to the robot
 * @param ball The velocity of the ball
 * @param max_robot_vel
 * @param alpha_factor An additional factor the angle is multiplied with (can be used to manually adjust the angle)
 * @return double
 */
bool isInsideAngle(const Eigen::Vector2d& ball_robot_vec, const Eigen::Vector2d& ball_vel, const double max_robot_vel,
                   const double alpha_factor);

/**
 * @brief Returns the goal point of the given teams enemy team
 *
 * @param team The team of whose enemy we want the goal point
 * @param wm Thw World Model
 * @return Eigen::Vector2d The Teams enemy goal point
 */
Eigen::Vector2d getTeamsEnemyGoalPoint(const Team team, const transform::WorldModel& wm);

/**
 * @brief Returns the field size as a vector
 *
 * @param wm The World Model
 * @return Eigen::Vector2d (width, height) of the field
 */
Eigen::Vector2d getFieldSize(const transform::WorldModel& wm);
}  // namespace luhsoccer::observer::calculation::misc