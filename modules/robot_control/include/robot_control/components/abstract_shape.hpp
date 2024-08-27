#pragma once

#include "robot_control/components/abstract_component.hpp"
namespace luhsoccer::robot_control {
struct ComponentData;

/**
 * @brief The vector to a shape and the current relative velocity of obstacle and robot
 *
 */
struct VectorWithVelocityStamped {
    time::TimePoint stamp;
    std::optional<Eigen::Vector2d> vec;
    std::optional<Eigen::Vector2d> velocity;
};
constexpr double DEFAULT_BRIM_SPACING = 0.1;

/**
 * @brief Base shape class of features
 * This class implements a base shape type that define the geometric shape of all features
 */
class AbstractShape : public AbstractComponent {
   public:
    ~AbstractShape() override = default;
    AbstractShape(const AbstractShape&) = default;
    AbstractShape& operator=(const AbstractShape&) = default;
    AbstractShape(AbstractShape&&) = default;
    AbstractShape& operator=(AbstractShape&&) = default;

    /**
     * @brief calculate the vector and relative velocity to the closest point on this shape
     *
     * @param wm WorldModel
     * @param td TaskData
     * @param reference_position position to which the closest point is calculated
     * @param time time point
     * @param observe_position coordinate system in which the vector and the relative velocity is given. Only the
     * rotation is relevant
     * @return VectorWithVelocityStamped Vector and relative velocity to the shape
     */
    [[nodiscard]] virtual VectorWithVelocityStamped getTransformToClosestPoint(
        const ComponentData& comp_data, const transform::Position& robot_position,
        const transform::Position& observe_position = "") const = 0;

    /**
     * @brief return the center of this shape
     *
     * @param wm WorldModel
     * @param td TaskData
     * @param time time point
     * @param observe_position coordinate system in which the vector is given. Only the rotation is relevant
     * @return ComponentPosition
     */
    [[nodiscard]] virtual std::optional<Eigen::Vector2d> getCenter(
        const ComponentData& /*comp_data*/, const transform::Position& /*observe_position*/ = "") const {
        return std::nullopt;
    };

    [[nodiscard]] virtual bool hasArea(const ComponentData& /*comp_data*/) const { return false; }

    [[nodiscard]] virtual double getAreaSize(const ComponentData& /*comp_data*/) const { return 0.0; }

    [[nodiscard]] virtual std::vector<Eigen::Vector2d> getPointsOnBrim(
        const ComponentData& /*comp_data*/, double /*spacing*/ = DEFAULT_BRIM_SPACING,
        const transform::Position& /*observe_position*/ = "", double /*margin*/ = 0.0) const {
        return {};
    };

    [[nodiscard]] virtual bool isPointInArea(const Eigen::Vector2d& /*point*/, const ComponentData& /*comp_data*/,
                                             const transform::Position& /*observe_position*/ = "") const {
        return false;
    }

    [[nodiscard]] virtual bool isGroupable(const ComponentData& /*comp_data*/) const { return true; }

   protected:
    AbstractShape() = default;
};
}  // namespace luhsoccer::robot_control