#include <utility>

#include "observer/data_storage.hpp"

namespace luhsoccer::observer {

void DataStorage::addEmptyRobot(const RobotIdentifier& handle) {
    if (handle.isAlly()) {
        this->ally_robots[handle] = AllyRobot();
    } else {
        this->enemy_robots[handle] = EnemyRobot();
    }
}

void DataStorage::setBestPassReceiver(const RobotIdentifier& handle, transform::RobotHandle bpr_handle, double score) {
    if (handle.isEnemy()) {
        LOG_DEBUG(this->logger, "Enemy handle given to best-pass-receiver");
        return;
    }

    auto& ally = this->ally_robots[handle];
    ally.best_pass_receiver = {std::move(bpr_handle), score};
}

void DataStorage::setGoalProbability(const RobotIdentifier& handle, double score) {
    if (handle.isEnemy()) {
        LOG_DEBUG(this->logger, "Enemy handle given to goal-probability");
        return;
    }

    this->ally_robots[handle].goal_probability = score;
}

void DataStorage::setThreatLevel(const RobotIdentifier& handle, double threat_score) {
    if (handle.isAlly()) {
        LOG_DEBUG(this->logger, "Ally handle given to threat-level");
        return;
    }

    this->enemy_robots[handle].threat_score = threat_score;
}

void DataStorage::setPassDefended(const RobotIdentifier& handle, bool is_pass_defended) {
    if (handle.isAlly()) {
        LOG_DEBUG(this->logger, "Ally handle given to pass-defended");
        return;
    }

    this->enemy_robots[handle].pass_defended = is_pass_defended;
}

void DataStorage::setGoalDefended(const RobotIdentifier& handle, bool is_goal_defended) {
    if (handle.isAlly()) {
        LOG_DEBUG(this->logger, "Ally handle given to goal-defended");
        return;
    }

    this->enemy_robots[handle].goal_defended = is_goal_defended;
}

void DataStorage::setDefenseAreaDefended(const RobotIdentifier& handle, bool is_defense_area_defended) {
    if (handle.isAlly()) {
        LOG_DEBUG(this->logger, "Ally handle given to defense-area-defended");
        return;
    }

    this->enemy_robots[handle].defense_area_defended = is_defense_area_defended;
}

void DataStorage::setBallGoalProbability(double new_val) { this->ball_goal_probability = new_val; }

void DataStorage::setBallHolder(std::optional<BallHolder> new_robot) { this->ball_holder = std::move(new_robot); }

void DataStorage::setStrategyType(StrategyType strategy_type) { this->current_behaviour = strategy_type; }

void DataStorage::setLastBallToucher(std::optional<transform::RobotHandle> handle) {
    this->last_ball_toucher = std::move(handle);
}

void DataStorage::setBallTouchingForbiddenRobot(std::optional<transform::RobotHandle> handle) {
    this->ball_touch_forbidden_robot = std::move(handle);
}

}  // namespace luhsoccer::observer