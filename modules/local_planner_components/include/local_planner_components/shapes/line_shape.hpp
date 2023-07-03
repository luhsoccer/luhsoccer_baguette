

#pragma once

#include <memory>
#include <utility>

#include "local_planner/skills/abstract_shape.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::local_planner {

class LineShape : public AbstractShape {
   public:
    /**
     * @brief Construct a new Line Shape object
     *
     * @param start
     * @param end
     * @param cutoff_start makes the line longer/shorter by the given length (negativ for longer, positiv for shorter)
     * @param cutoff_end makes the line longer/shorter by the given length (negativ for longer, positiv for shorter)
     */
    LineShape(ComponentPosition start, ComponentPosition end, DoubleComponentParam cutoff_start = {0.0},
              DoubleComponentParam cutoff_end = {0.0})
        : start(std::move(start)),
          end(std::move(end)),
          cutoff_start(std::move(cutoff_start)),
          cutoff_end(std::move(cutoff_end)){};

    /**
     * @brief Construct a new Line Shape object given a point and an angle
     *
     * @param point A point on the line
     * @param angle The angle of the line
     * @param length The length of the line, default is 1
     * @param cutoff_start Cutoff for point
     * @param cutoff_end Cuttoff for the end point, which has a distance of length to point
     */
    LineShape(const ComponentPosition& point, const DoubleComponentParam& angle, const DoubleComponentParam& length,
              DoubleComponentParam cutoff_start = {0.0}, DoubleComponentParam cutoff_end = {0.0})
        : start(point),
          end{CALLBACK,
              [point, angle, length](const std::shared_ptr<const transform::WorldModel>& wm,
                                     const TaskData& td) -> transform::Position {
                  double angle_val = angle.val(wm, td);
                  Eigen::Vector2d dir(std::cos(angle_val), std::sin(angle_val));
                  dir.normalize();

                  dir *= length.val(wm, td);

                  std::optional<Eigen::Affine2d> translation_point =
                      point.positionObject(wm, td).getCurrentPosition(wm);

                  if (!translation_point.has_value()) return {""};

                  return {"", translation_point->translation().x() + dir.x(),
                          translation_point->translation().y() + dir.y()};
              }},
          cutoff_start(std::move(cutoff_start)),
          cutoff_end(std::move(cutoff_end)){};

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
    ComponentPosition start;
    ComponentPosition end;
    DoubleComponentParam cutoff_start;
    DoubleComponentParam cutoff_end;
};

}  // namespace luhsoccer::local_planner