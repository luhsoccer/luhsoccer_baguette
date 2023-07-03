

#pragma once

#include <memory>
#include <utility>

#include "local_planner/skills/abstract_shape.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::local_planner {

class CircleShape : public AbstractShape {
   public:
    CircleShape(ComponentPosition position, DoubleComponentParam radius, BoolComponentParam filled)
        : position(std::move(position)), radius(std::move(radius)), filled(std::move(filled)){};

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
    ComponentPosition position;
    DoubleComponentParam radius;
    BoolComponentParam filled;
};

}  // namespace luhsoccer::local_planner