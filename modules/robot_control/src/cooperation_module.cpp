#include "cooperation_module.hpp"
#include <numeric>
#include "config/robot_control_visualization_config.hpp"
#include "marker_adapter.hpp"
#include "robot_control/components/component_data.hpp"
#include "config/robot_control_config.hpp"
#include "robot_control/components/abstract_feature.hpp"
#include "transform/world_model.hpp"
#include "utils/utils.hpp"

namespace luhsoccer::robot_control {

CooperationModule::CooperationModule() {
    for (const auto& robot : transform::RobotDataStorage::generatePossibleRobots(MAX_ROBOTS_PER_TEAM, Team::ALLY)) {
        auto [it, success] = this->rotation_vectors.emplace(std::piecewise_construct, std::forward_as_tuple(robot),
                                                            std::forward_as_tuple());
        if (!success)
            throw std::runtime_error(
                "Could not emplace new robot vector storage, even though i did not existed before!");
    }
}
std::vector<bool> CooperationModule::getRotationVectors(
    const ComponentData& comp_data, const std::vector<std::shared_ptr<const AbstractObstacle>>& features,
    const Eigen::Vector2d& goal_vec) {
    std::vector<std::shared_ptr<const AbstractObstacle>> ally_robot_features;
    std::vector<std::shared_ptr<const AbstractObstacle>> other_features;

    for (const auto& feature : features) {
        auto robot = feature->getRobot();
        if (robot.has_value() && robot->isAlly()) {
            ally_robot_features.push_back(feature);
        } else {
            other_features.push_back(feature);
        }
    }
    auto robot_pos =
        transform::Position(comp_data.robot.getFrame()).getCurrentPosition(comp_data.wm, "", comp_data.time);
    if (!robot_pos.has_value()) return std::vector<bool>(features.size());

    Eigen::Vector2d robot_translation = robot_pos->translation();
    auto ally_robot_rotation_vectors =
        this->assignRotationVectorsToAllies(comp_data, ally_robot_features, goal_vec, robot_translation);

    auto other_rotation_vectors =
        this->assignRotationVectorsToOthers(comp_data, other_features, goal_vec, robot_translation);

    std::vector<bool> vectors(features.size());

    std::map<size_t, size_t> uid_to_index;
    int i = 0;
    for (const auto& feature : features) {
        uid_to_index[feature->getUid()] = i++;
    }

    for (const auto& element : ally_robot_rotation_vectors) {
        vectors[uid_to_index[element.first]] = element.second;
    }
    for (const auto& element : other_rotation_vectors) {
        vectors[uid_to_index[element.first]] = element.second;
    }

    return vectors;
}

std::vector<std::pair<size_t, bool>> CooperationModule::assignRotationVectorsToAllies(
    const ComponentData& comp_data, const std::vector<std::shared_ptr<const AbstractObstacle>>& ally_robot_features,
    const Eigen::Vector2d& goal_vec, const Eigen::Vector2d& robot_translation) {
    double angle_goal = std::atan2(goal_vec.y(), goal_vec.x());

    time::TimePoint time = comp_data.time;
    if (time.asNSec() == 0) {
        time = time::now();
    }

    std::vector<std::pair<size_t, bool>> ally_robot_rotation_vectors(ally_robot_features.size());

    auto assign_vector =
        [&](const std::shared_ptr<const AbstractObstacle>& ally_robot_feature) -> std::pair<size_t, bool> {
        // calc vector
        auto feature_vec = ally_robot_feature->getShape()->getCenter(comp_data);
        if (!feature_vec.has_value()) return {ally_robot_feature->getUid(), false};
        Eigen::Vector2d robot_feature_vec = feature_vec.value() - robot_translation;

        // just use false if not in influence distance
        if (robot_feature_vec.norm() > std::min(goal_vec.norm(), ally_robot_feature->getInfluenceDistance(comp_data)))
            return {ally_robot_feature->getUid(), false};

        double angle_feature = std::atan2(robot_feature_vec.y(), robot_feature_vec.x());

        double angle_group_goal = angle_goal - angle_feature;

        angle_group_goal = cropAngle(angle_group_goal);
        bool rotation_vector_upwards = angle_group_goal < 0.0;
        auto other_robot = ally_robot_feature->getRobot();

        if (!other_robot.has_value() || !other_robot->isAlly())
            return {ally_robot_feature->getUid(), rotation_vector_upwards};
        auto res = this->hasRobotVectorAssignedForMe(comp_data.robot, other_robot.value(), time);

        if (res.has_value()) {
            return {ally_robot_feature->getUid(), res.value()};
        }

        auto res_publish =
            this->publishRobotVector(comp_data.robot, other_robot.value(), time, rotation_vector_upwards);
        if (res_publish.has_value()) {
            // write was not successful take other
            return {ally_robot_feature->getUid(), res_publish.value()};
        } else {
            // write was successful
            return {ally_robot_feature->getUid(), rotation_vector_upwards};
        }
    };
    std::transform(ally_robot_features.begin(), ally_robot_features.end(), ally_robot_rotation_vectors.begin(),
                   assign_vector);

    return ally_robot_rotation_vectors;
}

std::optional<bool> CooperationModule::hasRobotVectorAssignedForMe(const RobotIdentifier& me,
                                                                   const RobotIdentifier& other_robot,
                                                                   const time::TimePoint& time) const {
    const std::shared_lock read_lock_storage(this->rotation_vectors_mtx);

    // check other robot has rotation data
    auto other_rotation_data = this->rotation_vectors.find(other_robot);
    if (other_rotation_data == this->rotation_vectors.end()) return std::nullopt;
    const std::shared_lock read_lock_other_rotation_data(other_rotation_data->second.mtx);

    // check time
    if (time - other_rotation_data->second.time > this->vector_timeout_duration) return std::nullopt;
    // check rotation data for me
    auto my_rotation_data = other_rotation_data->second.rotation_vectors.find(me);
    if (my_rotation_data == other_rotation_data->second.rotation_vectors.end()) return std::nullopt;

    // check self defined
    if (!my_rotation_data->second.self_defined) return std::nullopt;

    return my_rotation_data->second.upwards;
}

std::optional<bool> CooperationModule::publishRobotVector(const RobotIdentifier& me, const RobotIdentifier& other_robot,
                                                          const time::TimePoint& time, bool upwards) {
    // check i have rotation data
    std::unique_lock read_lock_storage(this->rotation_vectors_mtx);
    auto my_rotation_data = this->rotation_vectors.find(me);
    auto other_rotation_data = this->rotation_vectors.find(other_robot);

    if (my_rotation_data == this->rotation_vectors.end())
        throw std::runtime_error("Rotation data for robot does not exists!");
    if (other_rotation_data == this->rotation_vectors.end())
        throw std::runtime_error("Rotation data for robot does not exists!");

    std::shared_lock other_robot_read_lock(other_rotation_data->second.mtx, std::defer_lock);
    std::unique_lock my_rotation_data_read_lock(my_rotation_data->second.mtx, std::defer_lock);

    // needed to defer deadlock
    std::lock(my_rotation_data_read_lock, other_robot_read_lock);

    // check if other robot has value for me
    auto has_other_robot_vector_for_me = [&]() -> std::optional<bool> {
        // check time
        if (time - other_rotation_data->second.time > this->vector_timeout_duration) return std::nullopt;
        // check rotation data for me
        auto my_rotation_data = other_rotation_data->second.rotation_vectors.find(me);
        if (my_rotation_data == other_rotation_data->second.rotation_vectors.end()) return std::nullopt;

        // check self defined
        if (!my_rotation_data->second.self_defined) return std::nullopt;
        return my_rotation_data->second.upwards;
    };

    auto res = has_other_robot_vector_for_me();
    if (res.has_value()) {
        // dont write if other robot has value for me
        return res.value();
    }

    // write data
    my_rotation_data->second.rotation_vectors[other_robot] = RotationVectorState{true, upwards};
    my_rotation_data->second.time = time;
    return std::nullopt;
}

double distanceBetweenShapes(const ComponentData& comp_data, const std::shared_ptr<const AbstractShape>& shape1,
                             const std::shared_ptr<const AbstractShape>& shape2) {
    auto center1 = shape1->getCenter(comp_data);
    if (!center1.has_value()) return std::numeric_limits<double>::infinity();

    if (!robotControlConfig().obstacle_better_grouping) {
        auto center2 = shape2->getCenter(comp_data);
        if (!center2.has_value()) return std::numeric_limits<double>::infinity();

        return (center2.value() - center1.value()).norm();
    }

    Eigen::Vector2d close_point1 = center1.value();
    Eigen::Vector2d close_point2 = Eigen::Vector2d::Zero();

    std::unique_ptr<MarkerAdapter> ma = std::make_unique<MarkerAdapter>();
    ComponentData shape_comp_data(comp_data, *ma);

    constexpr int ITERATIONS = 1;
    for (int i = 0; i < ITERATIONS; i++) {
        auto res2 = shape2->getTransformToClosestPoint(shape_comp_data, {"", close_point1.x(), close_point1.y()}, "");

        if (!res2.vec.has_value()) return std::numeric_limits<double>::infinity();
        close_point2 = res2.vec.value() + close_point1;

        auto res1 = shape1->getTransformToClosestPoint(shape_comp_data, {"", close_point2.x(), close_point2.y()}, "");

        if (!res1.vec.has_value()) return std::numeric_limits<double>::infinity();
        close_point1 = res1.vec.value() + close_point2;
    }

    if (robotControlVisualizationConfig().display_obstacle_distance_vector) {
        marker::Line l("");
        l.setLinePoints({close_point1.x(), close_point1.y()}, {close_point2.x(), close_point2.y()});
        l.setColor(RC_RED);
        comp_data.ma.displayMarker(l);
    }

    return (close_point2 - close_point1).norm();
}

std::vector<std::pair<size_t, bool>> CooperationModule::assignRotationVectorsToOthers(
    const ComponentData& comp_data, const std::vector<std::shared_ptr<const AbstractObstacle>>& other_features,
    const Eigen::Vector2d& goal_vec, const Eigen::Vector2d& robot_translation) {
    double angle_goal = std::atan2(goal_vec.y(), goal_vec.x());

    // find groups

    std::vector<size_t> other_feature_indexes(other_features.size());
    std::iota(other_feature_indexes.begin(), other_feature_indexes.end(), 0);

    std::function<std::vector<size_t>(size_t, std::vector<size_t>&,
                                      const std::vector<std::shared_ptr<const AbstractObstacle>>&, double)>
        find_neighbors =
            [&find_neighbors, &comp_data](size_t feature_index, std::vector<size_t>& feature_indexes,
                                          const std::vector<std::shared_ptr<const AbstractObstacle>>& features,
                                          double neighbour_distance) -> std::vector<size_t> {
        std::vector<size_t> neighbour_indexes;
        for (size_t i = 0; i < feature_indexes.size(); i++) {
            if (!features[feature_indexes[i]]->getShape()->isGroupable(comp_data)) continue;
            double distance = distanceBetweenShapes(comp_data, features[feature_index]->getShape(),
                                                    features[feature_indexes[i]]->getShape());
            if (distance < neighbour_distance) {
                size_t neighbour_index = feature_indexes[i];
                neighbour_indexes.push_back(neighbour_index);
                feature_indexes.erase(std::next(feature_indexes.begin(), static_cast<int>(i)));
                i--;
                std::vector<size_t> neighbour_neighbors =
                    find_neighbors(neighbour_index, feature_indexes, features, neighbour_distance);
                neighbour_indexes.insert(neighbour_indexes.end(), neighbour_neighbors.begin(),
                                         neighbour_neighbors.end());
            }
        }
        return neighbour_indexes;
    };

    std::vector<std::vector<size_t>> groups;
    double neighbour_distance = robotControlConfig().obstacle_grouping_distance;
    while (other_feature_indexes.size() > 0) {
        size_t origin_index = other_feature_indexes.front();
        other_feature_indexes.erase(other_feature_indexes.begin());
        auto group = find_neighbors(origin_index, other_feature_indexes, other_features, neighbour_distance);
        group.push_back(origin_index);
        groups.push_back(group);
    }

    std::vector<std::pair<size_t, bool>> other_rotation_vectors(other_features.size());
    for (const auto& group_indexes : groups) {
        Eigen::Vector2d weighted_group_center = {0.0, 0.0};
        Eigen::Vector2d unweighted_group_center = {0.0, 0.0};
        size_t group_counter = 0;
        double group_area = 0.0;

        for (size_t index : group_indexes) {
            auto feature_vec = other_features[index]->getShape()->getCenter(comp_data);
            double area = other_features[index]->getShape()->getAreaSize(comp_data);
            if (feature_vec.has_value()) {
                weighted_group_center += (feature_vec.value() - robot_translation) * area;
                unweighted_group_center += feature_vec.value() - robot_translation;
                group_counter++;
                group_area += std::abs(area);
            }
        }
        if (group_counter != 0) {
            Eigen::Vector2d group_center = {0.0, 0.0};
            if (group_area != 0.0 && robotControlConfig().obstacle_better_grouping) {
                group_center = weighted_group_center / group_area;
            } else {
                group_center = unweighted_group_center / static_cast<double>(group_counter);
            }

            double angle_feature = std::atan2(group_center.y(), group_center.x());

            double angle_group_goal = angle_goal - angle_feature;

            angle_group_goal = cropAngle(angle_group_goal);
            bool rotation_vector = angle_group_goal < 0.0;
            // if (std::abs(angle_group_goal) > L_PI / 2.0) rotation_vector = !rotation_vector;

            for (size_t index : group_indexes) {
                other_rotation_vectors[index] = {other_features[index]->getUid(), rotation_vector};
            }

            if (robotControlVisualizationConfig().display_obstacle_group_centers) {
                Eigen::Vector2d global_group_center = robot_translation + group_center;
                marker::Circle c({"", global_group_center.x(), global_group_center.y(), 0.0});
                c.setColor(RC_ORANGE);
                constexpr double CENTER_RADIUS = 0.025;
                c.setRadius(CENTER_RADIUS);
                static const Eigen::Vector2d TEXT_OFFSET = {-0.1, 0.1};
                marker::Text t(
                    {"", global_group_center.x() + TEXT_OFFSET.x(), global_group_center.y() + TEXT_OFFSET.y(), 0.0});
                t.setText(std::to_string(group_area));
                t.setColor(RC_ORANGE);
                comp_data.ma.displayMarker(c);
                comp_data.ma.displayMarker(t);
            }
        }
    }
    return other_rotation_vectors;
}
}  // namespace luhsoccer::robot_control