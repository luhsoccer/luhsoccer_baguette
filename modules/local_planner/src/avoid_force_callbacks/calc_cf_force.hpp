#include <Eigen/Geometry>

namespace luhsoccer::local_planner {

Eigen::Vector2d calcCFForce(const Eigen::Vector2d& vec_to_feature, const Eigen::Vector2d& relative_velocity,
                            bool rotation_vector_upwards);

double calcTargetRelaxationFactors(const std::optional<Eigen::Vector2d>& min_obstacle_vec,
                                   const Eigen::Vector2d& max_target_vec);

}  // namespace luhsoccer::local_planner