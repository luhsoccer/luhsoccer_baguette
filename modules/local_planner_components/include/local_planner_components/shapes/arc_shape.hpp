
#pragma once

#include <memory>
#include <utility>

#include "local_planner/skills/abstract_shape.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::local_planner {

class ArcShape : public AbstractShape {
   public:
    ArcShape(ComponentPosition center, DoubleComponentParam radius, DoubleComponentParam local_angle)
        : center(std::move(center)), radius(std::move(radius)), local_angle(std::move(local_angle)){};

    [[nodiscard]] VectorWithVelocityStamped getTransformToClosestPoint(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
        const ComponentPosition& reference_position, time::TimePoint time = time::TimePoint(0),
        const ComponentPosition& observe_position = {""}) const override;

    [[nodiscard]] std::optional<Eigen::Vector2d> getCenter(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
        time::TimePoint time = time::TimePoint(0), const ComponentPosition& observe_position = {""}) const override;

    [[nodiscard]] std::vector<Marker> getVisualizationMarker(const std::shared_ptr<const transform::WorldModel>& wm,
                                                             const TaskData& td,
                                                             time::TimePoint time = time::TimePoint(0)) const override;

   private:
    [[nodiscard]] std::optional<std::pair<double, Eigen::Vector2d>> defineStart(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
        time::TimePoint time = time::TimePoint(0), const ComponentPosition& observe_position = {""}) const;

    [[nodiscard]] std::optional<std::pair<double, Eigen::Vector2d>> defineEnd(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
        time::TimePoint time = time::TimePoint(0), const ComponentPosition& observe_position = {""}) const;
    ComponentPosition center;
    DoubleComponentParam radius;
    DoubleComponentParam local_angle;
};

}  // namespace luhsoccer::local_planner