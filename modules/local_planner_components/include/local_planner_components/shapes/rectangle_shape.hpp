

#pragma once

#include <memory>
#include <utility>

#include "local_planner/skills/abstract_shape.hpp"
#include "local_planner/skills/skill_util.hpp"
#include "local_planner_components/shapes/line_shape.hpp"

namespace luhsoccer::local_planner {

class RectangleShape : public AbstractShape {
   public:
    /**
     * @brief Construct a new Rectangle Shape object given the center and height and width
     *
     * @param cs
     * @param center
     * @param width
     * @param height
     * @param filled If the reference is in a filled rect, the vector will be (0,0)
     */
    RectangleShape(const ComponentPosition& center, const DoubleComponentParam& width,
                   const DoubleComponentParam& height, BoolComponentParam filled)
        : p1(CALLBACK,
             [center, height, width](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) {
                 Eigen::Affine2d offest = center.positionObject(wm, td).getPositionOffset();
                 Eigen::Affine2d rectangle_offset =
                     Eigen::Translation2d(height.val(wm, td) / 2, width.val(wm, td) / 2) * Eigen::Rotation2Dd(0);
                 return transform::Position(center.positionObject(wm, td).getFrame(), offest * rectangle_offset);
             }),
          p2(CALLBACK,
             [center, height, width](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) {
                 Eigen::Affine2d offest = center.positionObject(wm, td).getPositionOffset();
                 Eigen::Affine2d rectangle_offset =
                     Eigen::Translation2d(height.val(wm, td) / 2, -width.val(wm, td) / 2) * Eigen::Rotation2Dd(0);
                 return transform::Position(center.positionObject(wm, td).getFrame(), offest * rectangle_offset);
             }),
          p3(CALLBACK,
             [center, height, width](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) {
                 Eigen::Affine2d offest = center.positionObject(wm, td).getPositionOffset();
                 Eigen::Affine2d rectangle_offset =
                     Eigen::Translation2d(-height.val(wm, td) / 2, -width.val(wm, td) / 2) * Eigen::Rotation2Dd(0);
                 return transform::Position(center.positionObject(wm, td).getFrame(), offest * rectangle_offset);
             }),
          p4(CALLBACK,
             [center, height, width](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) {
                 Eigen::Affine2d offest = center.positionObject(wm, td).getPositionOffset();
                 Eigen::Affine2d rectangle_offset =
                     Eigen::Translation2d(-height.val(wm, td) / 2, width.val(wm, td) / 2) * Eigen::Rotation2Dd(0);
                 return transform::Position(center.positionObject(wm, td).getFrame(), offest * rectangle_offset);
             }),
          l1(p1, p2),
          l2(p2, p3),
          l3(p3, p4),
          l4(p4, p1),
          filled(std::move(filled)),
          center(center),
          height(height),
          width(width){};

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
    // RectangleShapeData data;
    ComponentPosition p1;
    ComponentPosition p2;
    ComponentPosition p3;
    ComponentPosition p4;
    LineShape l1;
    LineShape l2;
    LineShape l3;
    LineShape l4;
    BoolComponentParam filled;
    ComponentPosition center;
    DoubleComponentParam height;
    DoubleComponentParam width;
};

}  // namespace luhsoccer::local_planner