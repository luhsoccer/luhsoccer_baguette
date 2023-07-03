#pragma once

#include "local_planner/skills/skill_util.hpp"
#include "marker_service/marker.hpp"
#include "local_planner/skills/abstract_component.hpp"
namespace luhsoccer::local_planner {
/**
 * @brief The vector to a shape and the current relative velocity of obstacle and robot
 *
 */
struct VectorWithVelocityStamped {
    time::TimePoint stamp;
    std::optional<Eigen::Vector2d> vec;
    std::optional<Eigen::Vector2d> velocity;
};

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
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
        const ComponentPosition& reference_position, time::TimePoint time = time::TimePoint(0),
        const ComponentPosition& observe_position = {""}) const = 0;

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
        const std::shared_ptr<const transform::WorldModel>& /*wm*/, const TaskData& /*td*/,
        time::TimePoint /*time*/ = time::TimePoint(0), const ComponentPosition& /*observe_position*/ = {""}) const {
        return std::nullopt;
    };

    /**
     * @brief Get the current Visualization Marker
     *
     * @param wm WorldModel
     * @param td TaskData
     * @param time time point
     * @return Marker Visualization Marker
     */
    [[nodiscard]] virtual std::vector<Marker> getVisualizationMarker(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
        time::TimePoint time = time::TimePoint(0)) const = 0;

   protected:
    /**
     * @brief Construct a new Abstract Shape object
     *
     */
    AbstractShape() = default;
};
}  // namespace luhsoccer::local_planner