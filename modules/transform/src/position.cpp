#include "transform/position.hpp"

namespace luhsoccer::transform {

[[nodiscard]] std::optional<Eigen::Affine2d> Position::getCurrentPosition(const std::shared_ptr<const WorldModel>& wm,
                                                                          const Position& reference_position,
                                                                          const time::TimePoint& time) const {
    if (!wm) return std::nullopt;

    if (callback.has_value()) return callback.value()(wm, time).getCurrentPosition(wm, reference_position, time);

    std::optional<Transform> parent_transform = wm->getTransform(this->frame, reference_position.getFrame(), time);
    if (!parent_transform) return std::nullopt;

    return Eigen::Affine2d(reference_position.getPositionOffset().matrix().inverse() *
                           parent_transform->transform.matrix() * position.matrix());
}

[[nodiscard]] std::optional<Eigen::Vector3d> Position::getVelocity(const std::shared_ptr<const WorldModel>& wm,
                                                                   const Position& other_position,
                                                                   const Position& observation_position,
                                                                   const time::TimePoint& time,
                                                                   const time::Duration& averaging_interval) const {
    if (!wm) return std::nullopt;

    if (callback.has_value())
        return callback.value()(wm, time).getVelocity(wm, other_position, observation_position, time,
                                                      averaging_interval);

    std::optional<Velocity> parent_velocity = wm->getVelocity(
        this->frame, other_position.getFrame(), observation_position.getFrame(), time, averaging_interval);
    if (!parent_velocity) return std::nullopt;
    // ToDo consider position in frame -> see tm3

    return parent_velocity->velocity;
}

[[nodiscard]] std::string Position::getString() const {
    double angle = Eigen::Rotation2Dd(this->position.rotation()).angle();
    std::string frame = this->frame;
    if (frame.empty()) {
        frame = "World";
    }
    std::string name = fmt::format("({:0.3f}, {:0.3f}, {:0.3f}°)@{}", this->position.translation().x(),
                                   this->position.translation().y(), angle, frame);

    return name;
}

}  // namespace luhsoccer::transform