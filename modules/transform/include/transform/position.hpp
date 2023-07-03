#pragma once

#include <Eigen/Geometry>
#include <optional>
#include <utility>

#include "transform/world_model.hpp"

namespace luhsoccer::transform {
class Position {
    using Callback = std::function<Position(const std::shared_ptr<const WorldModel>& wm, const time::TimePoint& time)>;

   public:
    /**
     * @brief Construct a new Position in world model
     *
     * @param wm reference to world model object
     * @param frame name of frame in world model
     * @param position position in @p frame of world model
     */
    inline Position(std::string frame, Eigen::Affine2d position) noexcept
        : position(std::move(position)), frame(std::move(frame)), callback(std::nullopt) {}

    /**
     * @brief Construct a new Position in world model
     *
     * @param wm reference to world model object
     * @param frame name of frame in world model
     * @param x x coordinate of position in @p frame
     * @param y y coordinate of position in @p frame
     * @param t rotation of position
     */
    inline Position(std::string frame, double x = 0.0, double y = 0.0, double t = 0.0) noexcept
        : position(Eigen::Translation2d(x, y) * Eigen::Rotation2Dd(t)),
          frame(std::move(frame)),
          callback(std::nullopt) {}

    /// @brief this constructor exists so that a frame name can be implicitly converted to a position
    Position(const char* frame) : Position(frame, 0.0){};

    Position(Callback callback) : position(), frame(), callback(callback){};

    /**
     * @brief Get the Current Position in @p frame
     *
     * @param frame frame the position is given in, empty for world frame
     * @param time time at which the position is given, time::TimePoint(0) for latest
     * @return std::optional<Eigen::Affine2d>  position in @p frame
     */
    [[nodiscard]] std::optional<Eigen::Affine2d> getCurrentPosition(
        const std::shared_ptr<const WorldModel>& wm, const Position& reference_position = "",
        const time::TimePoint& time = time::TimePoint(0)) const;

    /**
     * @brief Get the Velocity of fame of this position
     *
     * @param other_frame the relative velocity to this frame will be returned if possible
     * @param observation_frame frame in which the velocity is given in
     * @param time point in time at which the velocity is given
     * @return std::optional<Eigen::Vector3d> velocity of frame of position
     */
    [[nodiscard]] std::optional<Eigen::Vector3d> getVelocity(const std::shared_ptr<const WorldModel>& wm,
                                                             const Position& other_position = "",
                                                             const Position& observation_position = "",
                                                             const time::TimePoint& time = time::TimePoint(0)) const;

    /// set the position in given frame
    void setPosition(const Eigen::Affine2d& position) { this->position = position; };

    /// @brief get the offset of this position from frame
    [[nodiscard]] Eigen::Affine2d getPositionOffset() const { return position; }

    /// get frame of position
    [[nodiscard]] inline std::string getFrame() const { return frame; };

   private:
    /// position in frame
    Eigen::Affine2d position;

    /// frame of world model
    std::string frame;

    std::optional<Callback> callback;
};

}  // namespace luhsoccer::transform