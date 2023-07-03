
#include "transform_helper/world_model_helper.hpp"

namespace luhsoccer::transform::helper {

[[nodiscard]] std::optional<Eigen::Vector2d> getPosition(const transform::RobotHandle& handle,
                                                         const time::TimePoint time) {
    const auto tf_pos = handle.getPosition();
    const auto world_model = handle.getWorldModel().lock();
    if (world_model == nullptr) return std::nullopt;

    const auto affine = tf_pos.getCurrentPosition(world_model, world_model->getGlobalFrame(), time);
    if (!affine.has_value()) return std::nullopt;

    return affine->translation();
}

[[nodiscard]] std::optional<Eigen::Vector3d> getPositionAndRotation(const transform::RobotHandle& handle,
                                                                    const time::TimePoint time) {
    const auto tf_pos = handle.getPosition();
    const auto world_model = handle.getWorldModel().lock();
    if (world_model == nullptr) return std::nullopt;

    const auto affine = tf_pos.getCurrentPosition(world_model, world_model->getGlobalFrame(), time);
    if (!affine.has_value()) return std::nullopt;

    // make a 3d vector out of the 2d vector and the angle
    Eigen::Vector3d result;
    result << affine->translation(), Eigen::Rotation2Dd(affine->rotation()).angle();
    return result;
}

[[nodiscard]] std::optional<Eigen::Vector3d> getVelocity(const transform::RobotHandle& handle,
                                                         const time::TimePoint time) {
    const auto tf_pos = handle.getPosition();
    const auto world_model = handle.getWorldModel().lock();
    if (world_model == nullptr) return std::nullopt;

    auto vel = tf_pos.getVelocity(world_model, world_model->getGlobalFrame(), world_model->getGlobalFrame(), time);
    return vel;
}

[[nodiscard]] std::optional<Eigen::Vector2d> getBallPosition(const transform::WorldModel& wm,
                                                             const time::TimePoint time) {
    const auto tf = wm.getTransform(wm.getBallFrame(), wm.getGlobalFrame(), time);
    if (!tf.has_value()) return std::nullopt;
    return tf->transform.translation();
}

[[nodiscard]] std::optional<Eigen::Vector3d> getBallVelocity(const transform::WorldModel& wm,
                                                             const time::TimePoint time) {
    const auto vel = wm.getVelocity(wm.getBallFrame(), wm.getGlobalFrame(), wm.getGlobalFrame(), time);

    if (!vel.has_value()) return std::nullopt;

    return vel->velocity;
}

}  // namespace luhsoccer::transform::helper