#include "local_planner/avoidance_manager.hpp"
#include "avoidance_manager_units/side_decision.hpp"
#include "avoidance_manager_units/cooperative_side_decision.hpp"
#include "avoidance_manager_units/haddadin.hpp"

#include "avoid_force_callbacks/ataka_callback.hpp"
#include "avoid_force_callbacks/apf_callback.hpp"
#include "avoid_force_callbacks/cf_force_callback.hpp"
#include "avoid_force_callbacks/gyroscopic_callback.hpp"

#include "local_planner/avoidance_manager_unit.hpp"

namespace luhsoccer::local_planner {

AvoidanceManager::AvoidanceManager()
    : magnetic_field_vector_mode(MagenticFieldVectorMode::COOPERATIVE_SIDE_DECISION),
      avoid_force_mode(AvoidForceMode::CIRCULAR_FIELDS) {
    this->avoidance_manager_units.emplace(MagenticFieldVectorMode::SIDE_DECISION, std::make_unique<SideDecision>());
    this->avoidance_manager_units.emplace(MagenticFieldVectorMode::COOPERATIVE_SIDE_DECISION,
                                          std::make_unique<CooperativeSideDecision>());
    this->avoidance_manager_units.emplace(MagenticFieldVectorMode::HADDADIN, std::make_unique<HaddadinDecision>());

    this->avoid_force_callbacks.emplace(AvoidForceMode::CIRCULAR_FIELDS, getCFForce);
    this->avoid_force_callbacks.emplace(AvoidForceMode::APF, getAPFForce);
    this->avoid_force_callbacks.emplace(AvoidForceMode::ATAKA, getAtakaForce);
    this->avoid_force_callbacks.emplace(AvoidForceMode::GYROSCOPIC, getGyroscopicForce);
}

std::vector<bool> AvoidanceManager::getRotationVectors(
    const std::vector<std::shared_ptr<const AbstractCFObstacle>>& features, const Eigen::Vector2d& goal_vec,
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
    time::TimePoint time) {
    const std::shared_lock lock(this->magentic_field_vector_mode_mtx);

    auto unit_it = this->avoidance_manager_units.find(this->magnetic_field_vector_mode);
    if (unit_it != this->avoidance_manager_units.end()) {
        return unit_it->second->getRotationVectors(features, goal_vec, wm, td, robot, time);
    } else {
        return std::vector<bool>(features.size());
    }
}

AvoidForceResult AvoidanceManager::getTotalForce(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
    const time::TimePoint time, const std::vector<std::shared_ptr<const AbstractCFObstacle>>& obstacles,
    const Eigen::Vector2d& target_force, const Eigen::Vector2d& max_target,
    const Eigen::Vector2d& mean_weighted_target) {
    const std::shared_lock lock(this->avoid_force_mode_mtx);
    auto callback_it = this->avoid_force_callbacks.find(this->avoid_force_mode);
    if (callback_it != this->avoid_force_callbacks.end()) {
        return callback_it->second(wm, td, robot, *this, time, obstacles, target_force, max_target,
                                   mean_weighted_target);
    } else {
        throw std::runtime_error("Current mode is not available!");
    }
}

void AvoidanceManager::setMagenticFieldVectorMode(const MagenticFieldVectorMode& mode) {
    const std::unique_lock lock(this->magentic_field_vector_mode_mtx);
    this->magnetic_field_vector_mode = mode;
}

MagenticFieldVectorMode AvoidanceManager::getMagneticFieldVectorMode() const {
    const std::shared_lock lock(this->magentic_field_vector_mode_mtx);
    return this->magnetic_field_vector_mode;
}

AvoidForceMode AvoidanceManager::getAvoidForceMode() const {
    const std::shared_lock lock(this->avoid_force_mode_mtx);
    return this->avoid_force_mode;
}

void AvoidanceManager::setAvoidForceMode(AvoidForceMode mode) {
    const std::unique_lock lock(this->avoid_force_mode_mtx);
    this->avoid_force_mode = mode;
}

}  // namespace luhsoccer::local_planner