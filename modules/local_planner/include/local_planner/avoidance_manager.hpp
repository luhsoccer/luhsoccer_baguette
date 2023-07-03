
#pragma once
#include <map>
#include "local_planner/skills/abstract_feature.hpp"
#include "local_planner/avoidance_manager_unit.hpp"
namespace luhsoccer::local_planner {

class AbstractAvoidanceManagerUnit;

enum class MagenticFieldVectorMode {
    OFF,
    MULTI_AGENT,
    SIDE_DECISION,
    COOPERATIVE_SIDE_DECISION,
    HADDADIN,
};

enum class AvoidForceMode { CIRCULAR_FIELDS, APF, ATAKA, GYROSCOPIC };

class AvoidanceManager {
   public:
    AvoidanceManager();

    std::vector<bool> getRotationVectors(const std::vector<std::shared_ptr<const AbstractCFObstacle>>& features,
                                         const Eigen::Vector2d& goal_vec,
                                         const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                                         const RobotIdentifier& robot, time::TimePoint time);
    void setMagenticFieldVectorMode(const MagenticFieldVectorMode& mode);
    [[nodiscard]] MagenticFieldVectorMode getMagneticFieldVectorMode() const;

    void setAvoidForceMode(AvoidForceMode mode);
    [[nodiscard]] AvoidForceMode getAvoidForceMode() const;

    AvoidForceResult getTotalForce(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                                   const RobotIdentifier& robot, const time::TimePoint time,
                                   const std::vector<std::shared_ptr<const AbstractCFObstacle>>& obstacles,
                                   const Eigen::Vector2d& target_force, const Eigen::Vector2d& max_target,
                                   const Eigen::Vector2d& mean_weighted_target);

   private:
    std::map<AvoidForceMode, std::function<AvoidForceResult(
                                 const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                                 const RobotIdentifier& robot, AvoidanceManager& am, const time::TimePoint time,
                                 const std::vector<std::shared_ptr<const AbstractCFObstacle>>& obstacles,
                                 const Eigen::Vector2d& target_force, const Eigen::Vector2d& max_target,
                                 const Eigen::Vector2d& mean_weighted_target)>>
        avoid_force_callbacks;

    std::map<MagenticFieldVectorMode, std::unique_ptr<AbstractAvoidanceManagerUnit>> avoidance_manager_units;
    MagenticFieldVectorMode magnetic_field_vector_mode;
    AvoidForceMode avoid_force_mode;
    mutable std::shared_mutex magentic_field_vector_mode_mtx;
    mutable std::shared_mutex avoid_force_mode_mtx;
};

}  // namespace luhsoccer::local_planner