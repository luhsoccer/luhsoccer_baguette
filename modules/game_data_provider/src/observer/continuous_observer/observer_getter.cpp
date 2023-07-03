
#include "observer/continuous_observer.hpp"

namespace luhsoccer::observer {

[[nodiscard]] std::optional<AllyRobot::BestPassReceiver> Observer::getBestPassReceiver(
    const RobotIdentifier& ally) const {
    return this->data_storage_buffer.getFrontBuffer().getBestPassReceiver(ally);
}

[[nodiscard]] std::optional<double> Observer::getGoalProbability(const RobotIdentifier& ally) const {
    return this->data_storage_buffer.getFrontBuffer().getGoalProbability(ally);
}

[[nodiscard]] std::optional<double> Observer::getThreatLevel(const RobotIdentifier& enemy) const {
    return this->data_storage_buffer.getFrontBuffer().getThreatLevel(enemy);
}

[[nodiscard]] double Observer::getBallGoalProbability() const {
    return this->data_storage_buffer.getFrontBuffer().getBallGoalProbability();
}

[[nodiscard]] std::optional<transform::RobotHandle> Observer::getBallCarrier() const {
    return this->data_storage_buffer.getFrontBuffer().getBallCarrier();
}

[[nodiscard]] std::optional<Team> Observer::getBallControllingTeam() const {
    return this->data_storage_buffer.getFrontBuffer().getBallControllingTeam();
}

[[nodiscard]] std::optional<Eigen::Vector2d> Observer::getBallObtainedPos() const {
    return this->data_storage_buffer.getFrontBuffer().getBallObtainedPos();
}

[[nodiscard]] StrategyType Observer::getStrategyType() const {
    return this->data_storage_buffer.getFrontBuffer().getStrategyType();
}

[[nodiscard]] std::optional<transform::RobotHandle> Observer::getLastBallTouchingRobot() const {
    return this->data_storage_buffer.getFrontBuffer().getLastBallToucher();
}

[[nodiscard]] std::optional<transform::RobotHandle> Observer::getBallTouchingForbiddenRobot() const {
    return this->data_storage_buffer.getFrontBuffer().getBallTouchingForbiddenRobot();
}

}  // namespace luhsoccer::observer