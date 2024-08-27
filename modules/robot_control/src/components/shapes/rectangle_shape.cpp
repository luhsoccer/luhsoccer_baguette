#include "robot_control/components/shapes/rectangle_shape.hpp"
#include "robot_control/components/component_data.hpp"
#include "marker_adapter.hpp"
#include "marker_service/marker.hpp"

namespace luhsoccer::robot_control {

RectangleShape::RectangleShape(const ComponentPosition& center, const DoubleComponentParam& width,
                               const DoubleComponentParam& height, BoolComponentParam filled)
    : p1(CALLBACK,
         [center, height, width](const ComponentData& comp_data, const ComponentUid&) {
             Eigen::Affine2d offset = center.positionObject(comp_data).getPositionOffset();
             Eigen::Affine2d rectangle_offset =
                 Eigen::Translation2d(height.val(comp_data) / 2, width.val(comp_data) / 2) * Eigen::Rotation2Dd(0);
             return transform::Position(center.positionObject(comp_data).getFrame(), offset * rectangle_offset);
         }),
      p2(CALLBACK,
         [center, height, width](const ComponentData& comp_data, const ComponentUid&) {
             Eigen::Affine2d offset = center.positionObject(comp_data).getPositionOffset();
             Eigen::Affine2d rectangle_offset =
                 Eigen::Translation2d(height.val(comp_data) / 2, -width.val(comp_data) / 2) * Eigen::Rotation2Dd(0);
             return transform::Position(center.positionObject(comp_data).getFrame(), offset * rectangle_offset);
         }),
      p3(CALLBACK,
         [center, height, width](const ComponentData& comp_data, const ComponentUid&) {
             Eigen::Affine2d offset = center.positionObject(comp_data).getPositionOffset();
             Eigen::Affine2d rectangle_offset =
                 Eigen::Translation2d(-height.val(comp_data) / 2, -width.val(comp_data) / 2) * Eigen::Rotation2Dd(0);
             return transform::Position(center.positionObject(comp_data).getFrame(), offset * rectangle_offset);
         }),
      p4(CALLBACK,
         [center, height, width](const ComponentData& comp_data, const ComponentUid&) {
             Eigen::Affine2d offset = center.positionObject(comp_data).getPositionOffset();
             Eigen::Affine2d rectangle_offset =
                 Eigen::Translation2d(-height.val(comp_data) / 2, width.val(comp_data) / 2) * Eigen::Rotation2Dd(0);
             return transform::Position(center.positionObject(comp_data).getFrame(), offset * rectangle_offset);
         }),
      lines({{p1, p2}, {p2, p3}, {p3, p4}, {p4, p1}}),
      filled(std::move(filled)),
      center(center),
      height(height),
      width(width){};

VectorWithVelocityStamped RectangleShape::getTransformToClosestPoint(
    const ComponentData& comp_data, const transform::Position& robot_position,
    const transform::Position& observe_position) const {
    VectorWithVelocityStamped trans_and_vel;
    trans_and_vel.stamp = comp_data.time;

    // translation
    std::optional<Eigen::Vector2d> min_vec;
    for (const LineShape& l : lines) {
        std::optional<Eigen::Vector2d> vec =
            l.getTransformToClosestPoint(comp_data, robot_position, observe_position).vec;
        if (vec.has_value() && (!min_vec.has_value() || vec->norm() < min_vec->norm())) min_vec = vec.value();
    }

    auto pos_in_center_frame =
        robot_position.getCurrentPosition(comp_data.wm, this->center.positionObject(comp_data), comp_data.time);

    if (!pos_in_center_frame.has_value()) return trans_and_vel;

    if (filled.val(comp_data) && abs(pos_in_center_frame->translation().x()) < this->height.val(comp_data) / 2 &&
        abs(pos_in_center_frame->translation().y()) < this->width.val(comp_data) / 2) {
        trans_and_vel.vec = {0.0, 0.0};
    } else {
        trans_and_vel.vec = min_vec;
    }

    // velocity
    std::optional<Eigen::Vector3d> velocity =
        this->p1.positionObject(comp_data).getVelocity(comp_data.wm, robot_position, observe_position, comp_data.time);
    if (velocity) {
        trans_and_vel.velocity = velocity->head(2);
    }

    // visualization
    marker::Rect m(this->center.positionObject(comp_data));
    m.setSize({this->height.val(comp_data), this->width.val(comp_data)});
    m.setFilled(this->filled.val(comp_data));
    comp_data.ma.displayMarker(m);

    return trans_and_vel;
}

std::optional<Eigen::Vector2d> RectangleShape::getCenter(const ComponentData& comp_data,
                                                         const transform::Position& observe_position) const {
    std::optional<Eigen::Affine2d> vec =
        this->center.positionObject(comp_data).getCurrentPosition(comp_data.wm, observe_position, comp_data.time);
    if (vec.has_value()) return vec->translation();
    return std::nullopt;
}

std::vector<Eigen::Vector2d> RectangleShape::getPointsOnBrim(const ComponentData& comp_data, double spacing,
                                                             const transform::Position& observe_position,
                                                             double margin) const {
    std::vector<Eigen::Vector2d> brim_points;

    auto center_pos =
        this->center.positionObject(comp_data).getCurrentPosition(comp_data.wm, observe_position, comp_data.time);
    if (!center_pos.has_value()) return {};
    Eigen::Vector2d center_trans = center_pos->translation();

    auto add_points_between_points = [&spacing, &brim_points, &comp_data, &observe_position, &margin, &center_trans](
                                         const ComponentPosition& start_position,
                                         const ComponentPosition& end_position) {
        auto start_res =
            start_position.positionObject(comp_data).getCurrentPosition(comp_data.wm, observe_position, comp_data.time);
        auto end_res =
            end_position.positionObject(comp_data).getCurrentPosition(comp_data.wm, observe_position, comp_data.time);

        if (!end_res.has_value() || !start_res.has_value()) return;

        Eigen::Vector2d start = start_res->translation();
        Eigen::Vector2d end = end_res->translation();

        auto add_margin = [&center_trans, &margin](Eigen::Vector2d& vec) {
            if (vec.x() < center_trans.x())
                vec.x() -= margin;
            else
                vec.x() += margin;
            if (vec.y() < center_trans.y())
                vec.y() -= margin;
            else
                vec.y() += margin;
        };
        add_margin(start);
        add_margin(end);

        std::vector<Eigen::Vector2d> p;
        int n_points = static_cast<int>((start - end).norm() / spacing);

        for (int i = 0; i < n_points; i++) {
            brim_points.emplace_back(start + (end - start) * i / n_points);
        }
    };
    add_points_between_points(this->p1, this->p2);
    add_points_between_points(this->p2, this->p3);
    add_points_between_points(this->p3, this->p4);
    add_points_between_points(this->p4, this->p1);
    return brim_points;
}

bool RectangleShape::isPointInArea(const Eigen::Vector2d& point, const ComponentData& comp_data,
                                   const transform::Position& observe_position) const {
    transform::Position point_position(observe_position.getFrame(), point.x(), point.y());

    auto pos_in_center_frame =
        point_position.getCurrentPosition(comp_data.wm, this->center.positionObject(comp_data), comp_data.time);

    if (!pos_in_center_frame.has_value()) return false;

    return abs(pos_in_center_frame->translation().x()) < this->height.val(comp_data) / 2 &&
           abs(pos_in_center_frame->translation().y()) < this->width.val(comp_data) / 2;
}

double RectangleShape::getAreaSize(const ComponentData& comp_data) const {
    if (this->filled.val(comp_data)) {
        return this->height.val(comp_data) * this->width.val(comp_data);
    }
    return 0.0;
}

}  // namespace luhsoccer::robot_control
