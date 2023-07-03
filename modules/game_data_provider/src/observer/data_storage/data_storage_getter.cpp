
#include "observer/data_storage.hpp"

namespace luhsoccer::observer {

[[nodiscard]] std::optional<AllyRobot::BestPassReceiver> DataStorage::getBestPassReceiver(
    const RobotIdentifier& ally) const {
    const auto data = this->ally_robots.find(ally);
    if (data == this->ally_robots.end()) return std::nullopt;

    return data->second.best_pass_receiver;
}

[[nodiscard]] std::optional<double> DataStorage::getGoalProbability(const RobotIdentifier& ally) const {
    const auto data = this->ally_robots.find(ally);
    if (data == this->ally_robots.end()) return std::nullopt;
    return data->second.goal_probability;
}

[[nodiscard]] std::optional<double> DataStorage::getThreatLevel(const RobotIdentifier& enemy) const {
    const auto data = this->enemy_robots.find(enemy);
    if (data == this->enemy_robots.end()) {
        return std::nullopt;
    }
    return data->second.threat_score;
}

[[nodiscard]] double DataStorage::getBallGoalProbability() const { return this->ball_goal_probability; }

[[nodiscard]] std::optional<transform::RobotHandle> DataStorage::getBallCarrier() const {
    if (!this->ball_holder.has_value()) return std::nullopt;
    return this->ball_holder->handle;
}

[[nodiscard]] std::optional<Team> DataStorage::getBallControllingTeam() const {
    if (!this->ball_holder.has_value()) return std::nullopt;
    return this->ball_holder->handle.getTeam();
}

[[nodiscard]] std::optional<Eigen::Vector2d> DataStorage::getBallObtainedPos() const {
    if (!this->ball_holder.has_value()) return std::nullopt;
    return this->ball_holder->ball_obtained_pos;
}

[[nodiscard]] StrategyType DataStorage::getStrategyType() const { return this->current_behaviour; }

[[nodiscard]] std::optional<transform::RobotHandle> DataStorage::getLastBallToucher() const {
    return this->last_ball_toucher;
}

[[nodiscard]] std::optional<transform::RobotHandle> DataStorage::getBallTouchingForbiddenRobot() const {
    return this->ball_touch_forbidden_robot;
}

}  // namespace luhsoccer::observer