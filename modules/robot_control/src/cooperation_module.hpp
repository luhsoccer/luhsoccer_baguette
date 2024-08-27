#pragma once

#include "core/robot_identifier.hpp"
#include "time/time.hpp"
namespace luhsoccer::robot_control {
class AbstractObstacle;
struct ComponentData;
class CooperationModule {
   public:
    CooperationModule();
    std::vector<bool> getRotationVectors(const ComponentData& comp_data,
                                         const std::vector<std::shared_ptr<const AbstractObstacle>>& features,
                                         const Eigen::Vector2d& goal_vec);

   private:
    std::vector<std::pair<size_t, bool>> assignRotationVectorsToAllies(
        const ComponentData& comp_data, const std::vector<std::shared_ptr<const AbstractObstacle>>& ally_robot_features,
        const Eigen::Vector2d& goal_vec, const Eigen::Vector2d& robot_translation);

    std::vector<std::pair<size_t, bool>> assignRotationVectorsToOthers(
        const ComponentData& comp_data, const std::vector<std::shared_ptr<const AbstractObstacle>>& other_features,
        const Eigen::Vector2d& goal_vec, const Eigen::Vector2d& robot_translation);

    std::optional<bool> hasRobotVectorAssignedForMe(const RobotIdentifier& me, const RobotIdentifier& other_robot,
                                                    const time::TimePoint& time) const;

    std::optional<bool> publishRobotVector(const RobotIdentifier& me, const RobotIdentifier& other_robot,
                                           const time::TimePoint& time, bool upwards);
    struct RotationVectorState {
        bool self_defined;
        bool upwards;
    };
    struct RotationVectorData {
        std::unordered_map<RobotIdentifier, RotationVectorState> rotation_vectors;
        time::TimePoint time;
        mutable std::shared_mutex mtx;
    };
    std::unordered_map<RobotIdentifier, RotationVectorData> rotation_vectors;
    mutable std::shared_mutex rotation_vectors_mtx;

    const time::Duration vector_timeout_duration{0.1};
};
}  // namespace luhsoccer::robot_control