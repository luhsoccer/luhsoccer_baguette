#include "side_decision.hpp"
#include <numeric>
#include "local_planner/skills/abstract_shape.hpp"
#include "utils/utils.hpp"

namespace luhsoccer::local_planner {

std::vector<bool> SideDecision::getRotationVectors(
    const std::vector<std::shared_ptr<const AbstractCFObstacle>>& features, const Eigen::Vector2d& goal_vec,
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
    time::TimePoint time) {
    /// calc vectors to features
    std::vector<std::optional<Eigen::Vector2d>> feature_vectors(features.size());
    std::transform(features.begin(), features.end(), feature_vectors.begin(),
                   [&](const std::shared_ptr<const AbstractCFObstacle>& feature) {
                       /// @todo get center of feature
                       auto robot_feature_vec = feature->getShape()->getTransformToClosestPoint(wm, td, robot, time);
                       if (robot_feature_vec.vec.has_value())
                           robot_feature_vec.vec.value() *=
                               1 + localPlannerConfig().robot_radius / robot_feature_vec.vec->norm();
                       return robot_feature_vec.vec;
                   });

    /// find groups
    std::vector<size_t> feature_indexes(features.size());
    std::iota(feature_indexes.begin(), feature_indexes.end(), 0);

    std::function<std::vector<size_t>(size_t, std::vector<size_t>&, const std::vector<std::optional<Eigen::Vector2d>>&,
                                      double neighbour_distance)>
        find_neighbors = [&find_neighbors](size_t feature_index, std::vector<size_t>& feature_indexes,
                                           const std::vector<std::optional<Eigen::Vector2d>>& feature_vectors,
                                           double neighbour_distance) -> std::vector<size_t> {
        if (!feature_vectors[feature_index].has_value()) return {};
        Eigen::Vector2d origin_vec = feature_vectors[feature_index].value();
        std::vector<size_t> neighbour_indexes;
        for (size_t i = 0; i < feature_indexes.size(); i++) {
            if (feature_vectors[feature_indexes[i]].has_value() &&
                (feature_vectors[feature_indexes[i]].value() - origin_vec).norm() < neighbour_distance) {
                size_t neighbour_index = feature_indexes[i];
                neighbour_indexes.push_back(neighbour_index);
                feature_indexes.erase(std::next(feature_indexes.begin(), static_cast<int>(i)));
                i--;
                std::vector<size_t> neighbour_neighbors =
                    find_neighbors(neighbour_index, feature_indexes, feature_vectors, neighbour_distance);
                neighbour_indexes.insert(neighbour_indexes.end(), neighbour_neighbors.begin(),
                                         neighbour_neighbors.end());
            }
        }
        return neighbour_indexes;
    };

    std::vector<std::vector<size_t>> groups;
    double neighbour_distance = localPlannerConfig().side_decision_neighbour_distance;
    while (feature_indexes.size() > 0) {
        size_t origin_index = feature_indexes.front();
        feature_indexes.erase(feature_indexes.begin());
        auto group = find_neighbors(origin_index, feature_indexes, feature_vectors, neighbour_distance);
        group.push_back(origin_index);
        groups.push_back(group);
    }

    std::vector<bool> rotation_vectors(features.size());

    double angle_goal = std::atan2(goal_vec.y(), goal_vec.x());

    for (const auto& group_indexes : groups) {
        /// @todo get center of feature
        Eigen::Vector2d group_center = {0.0, 0.0};
        size_t group_counter = 0;
        for (size_t index : group_indexes) {
            auto robot_feature_vec = features[index]->getShape()->getTransformToClosestPoint(wm, td, robot, time);
            if (robot_feature_vec.vec.has_value()) {
                group_center += robot_feature_vec.vec.value();
                group_counter++;
            }
        }
        if (group_counter != 0) {
            group_center /= static_cast<double>(group_counter);

            double angle_feature = std::atan2(group_center.y(), group_center.x());

            double angle_group_goal = angle_goal - angle_feature;

            angle_group_goal = cropAngle(angle_group_goal);
            bool rotation_vector = angle_group_goal < 0.0;
            for (size_t index : group_indexes) {
                rotation_vectors[index] = rotation_vector;
            }
        }
    }

    return rotation_vectors;
}
}  // namespace luhsoccer::local_planner