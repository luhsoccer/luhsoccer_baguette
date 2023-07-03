#include "haddadin.hpp"
#include <numeric>
#include "local_planner/skills/abstract_shape.hpp"
#include "utils/utils.hpp"

namespace luhsoccer::local_planner {

std::vector<bool> HaddadinDecision::getRotationVectors(
    const std::vector<std::shared_ptr<const AbstractCFObstacle>>& features, const Eigen::Vector2d& goal_vec,
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
    time::TimePoint time) {
    double angle_goal = std::atan2(goal_vec.y(), goal_vec.x());

    /// calc vectors to features
    std::vector<bool> rotation_vectors(features.size());
    std::transform(features.begin(), features.end(), rotation_vectors.begin(),
                   [&](const std::shared_ptr<const AbstractCFObstacle>& feature) {
                       auto res = feature->getShape()->getTransformToClosestPoint(wm, td, robot, time);
                       if (!res.vec.has_value()) return false;
                       res.vec.value() *= 1 + localPlannerConfig().robot_radius / res.vec->norm();

                       double angle_feature = std::atan2(res.vec->y(), res.vec->x());

                       double angle_group_goal = angle_goal - angle_feature;

                       angle_group_goal = cropAngle(angle_group_goal);
                       return angle_group_goal < 0.0;
                   });

    return rotation_vectors;
}
}  // namespace luhsoccer::local_planner