#include "cooperative_side_decision.hpp"
#include <numeric>
#include "local_planner/skills/abstract_shape.hpp"
#include "utils/utils.hpp"

namespace luhsoccer::local_planner {
std::vector<bool> CooperativeSideDecision::getRotationVectors(
    const std::vector<std::shared_ptr<const AbstractCFObstacle>>& features, const Eigen::Vector2d& goal_vec,
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
    time::TimePoint time) {
    time::TimePoint stamp = time;
    if (time == time::TimePoint(0)) stamp = time::now();

    std::vector<std::pair<size_t, std::shared_ptr<const AbstractCFObstacle>>> ally_robots_features;
    std::vector<std::pair<size_t, std::shared_ptr<const AbstractCFObstacle>>> other_features;

    for (size_t i = 0; i < features.size(); i++) {
        // skip feature if weight is zero
        constexpr double SMALL_VALUE = 0.00001;
        if (features[i]->getWeight(wm, td) < SMALL_VALUE) {
            continue;
        }

        auto other_robot = features[i]->getRobot();
        if (other_robot.has_value() && other_robot.value() == td.robot) {
            continue;
        } else if (other_robot.has_value() && other_robot->isAlly()) {
            ally_robots_features.emplace_back(i, features[i]);
        } else {
            other_features.emplace_back(i, features[i]);
        }
    }

    double angle_goal = std::atan2(goal_vec.y(), goal_vec.x());
    auto robot_transform = wm->getTransform(robot.getFrame(), "", time);
    if (!robot_transform.has_value()) return std::vector<bool>(features.size());

    // ally robots
    logger::Logger logger(fmt::format("CooperativeSideDecision{}", robot));
    {
        std::shared_lock storage_lock(this->rotation_vectors_mtx);
        auto my_vector_data = this->rotation_vectors.find(robot);
        if (my_vector_data == this->rotation_vectors.end()) {
            // add rotation vector store
            storage_lock.unlock();
            {
                std::unique_lock write_storage_lock(this->rotation_vectors_mtx);
                auto [it, success] = this->rotation_vectors.emplace(
                    std::piecewise_construct, std::forward_as_tuple(robot), std::forward_as_tuple());
                if (!success)
                    throw std::runtime_error(
                        "Could not emplace new robot vector storage, even though i did not existed before!");
            }
            storage_lock.lock();
        }
    }
    std::vector<std::pair<size_t, bool>> ally_robot_rotation_vectors(ally_robots_features.size());
    std::transform(
        ally_robots_features.begin(), ally_robots_features.end(), ally_robot_rotation_vectors.begin(),
        [&](const std::pair<size_t, std::shared_ptr<const AbstractCFObstacle>>& feature_element)
            -> std::pair<size_t, bool> {
            auto other_robot = feature_element.second->getRobot();
            if (other_robot && other_robot->isAlly()) {
                // check if robot already has rotation vector assigned for me
                auto other_rotation_data = this->rotation_vectors.find(other_robot.value());
                if (other_rotation_data != this->rotation_vectors.end()) {
                    // robot has data
                    std::shared_lock other_robot_data_lock(other_rotation_data->second.mtx);
                    auto other_robot_vector_for_me = other_rotation_data->second.rotation_vectors.find(robot);
                    if (other_robot_vector_for_me != other_rotation_data->second.rotation_vectors.end()) {
                        // robot has data of me
                        if (other_robot_vector_for_me->second.self_defined &&
                            (stamp - other_rotation_data->second.time) < this->vector_timeout_duration) {
                            // robot self defined data for me
                            // accept data
                            bool vector_upwards = other_robot_vector_for_me->second.upwards;
                            other_robot_data_lock.unlock();
                            auto my_vector_for_other_robot = this->rotation_vectors.find(robot);
                            std::unique_lock my_vector_write_lock(my_vector_for_other_robot->second.mtx);  // M0

                            my_vector_for_other_robot->second.rotation_vectors[other_robot.value()] =
                                RotationVectorState{false, vector_upwards};
                            my_vector_for_other_robot->second.time = stamp;

                            return {feature_element.first, vector_upwards};
                        }
                    }
                }
            }
            // calc vector
            auto feature_vec = feature_element.second->getShape()->getCenter(wm, td, time);
            if (!feature_vec.has_value()) return {feature_element.first, false};
            Eigen::Vector2d robot_feature_vec = feature_vec.value() - robot_transform->transform.translation();
            double angle_feature = std::atan2(robot_feature_vec.y(), robot_feature_vec.x());

            double angle_group_goal = angle_goal - angle_feature;

            angle_group_goal = cropAngle(angle_group_goal);
            bool rotation_vector_upwards = angle_group_goal < 0.0;

            if (other_robot && other_robot->isAlly()) {
                // check if own robot exists in rotation vectors
                std::shared_lock storage_lock(this->rotation_vectors_mtx);
                auto my_vector_data = this->rotation_vectors.find(robot);

                // again check
                auto other_rotation_data = this->rotation_vectors.find(other_robot.value());
                std::unique_lock my_vector_write_lock(my_vector_data->second.mtx, std::defer_lock);
                if (other_rotation_data != this->rotation_vectors.end()) {
                    // robot has data
                    std::shared_lock other_robot_read_lock(other_rotation_data->second.mtx, std::defer_lock);

                    // needed to defer deadlock
                    std::lock(my_vector_write_lock, other_robot_read_lock);
                    auto my_vector_it = other_rotation_data->second.rotation_vectors.find(robot);
                    if (my_vector_it != other_rotation_data->second.rotation_vectors.end()) {
                        // robot has data of me
                        if (my_vector_it->second.self_defined &&
                            (stamp - other_rotation_data->second.time) < this->vector_timeout_duration) {
                            // robot self defined data for me
                            return {feature_element.first, my_vector_it->second.upwards};
                        }
                    }
                } else {
                    my_vector_write_lock.lock();
                }
                my_vector_data->second.rotation_vectors[other_robot.value()] =
                    RotationVectorState{true, rotation_vector_upwards};
                my_vector_data->second.time = stamp;
            }

            return {feature_element.first, rotation_vector_upwards};
        });

    // other features

    /// calc vectors to features
    std::vector<std::optional<Eigen::Vector2d>> other_feature_vectors(other_features.size());
    std::transform(other_features.begin(), other_features.end(), other_feature_vectors.begin(),
                   [&](const std::pair<size_t, std::shared_ptr<const AbstractCFObstacle>>& feature_element)
                       -> std::optional<Eigen::Vector2d> {
                       auto feature_vec = feature_element.second->getShape()->getCenter(wm, td, time);
                       if (!feature_vec.has_value()) return std::nullopt;
                       return feature_vec.value() - robot_transform->transform.translation();
                   });

    /// find groups
    std::vector<size_t> other_feature_indexes(other_features.size());
    std::iota(other_feature_indexes.begin(), other_feature_indexes.end(), 0);

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
    while (other_feature_indexes.size() > 0) {
        size_t origin_index = other_feature_indexes.front();
        other_feature_indexes.erase(other_feature_indexes.begin());
        auto group = find_neighbors(origin_index, other_feature_indexes, other_feature_vectors, neighbour_distance);
        group.push_back(origin_index);
        groups.push_back(group);
    }

    std::vector<bool> other_rotation_vectors(other_features.size());

    for (const auto& group_indexes : groups) {
        Eigen::Vector2d group_center = {0.0, 0.0};
        size_t group_counter = 0;
        for (size_t index : group_indexes) {
            auto feature_vec = other_features[index].second->getShape()->getCenter(wm, td, time);

            if (feature_vec.has_value()) {
                group_center += feature_vec.value() - robot_transform->transform.translation();
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
                other_rotation_vectors[index] = rotation_vector;
            }
        }
    }

    std::vector<bool> rotation_vectors(features.size());
    for (auto ally_rot_vec : ally_robot_rotation_vectors) {
        rotation_vectors[ally_rot_vec.first] = ally_rot_vec.second;
    }
    for (size_t i = 0; i < other_rotation_vectors.size(); i++) {
        rotation_vectors[other_features[i].first] = other_rotation_vectors[i];
    }

    return rotation_vectors;
}
}  // namespace luhsoccer::local_planner