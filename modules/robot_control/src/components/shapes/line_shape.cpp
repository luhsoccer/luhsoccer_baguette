#include "robot_control/components/shapes/line_shape.hpp"

#include "robot_control/components/component_data.hpp"
#include "marker_adapter.hpp"
#include "marker_service/marker.hpp"

namespace luhsoccer::robot_control {

LineShape::LineShape(const ComponentPosition& point, const DoubleComponentParam& angle,
                     const DoubleComponentParam& length, DoubleComponentParam cutoff_start,
                     DoubleComponentParam cutoff_end)
    : start(point),
      end{CALLBACK,
          [point, angle, length](const ComponentData& comp_data, const ComponentUid&) -> transform::Position {
              double angle_val = angle.val(comp_data);
              Eigen::Vector2d dir(std::cos(angle_val), std::sin(angle_val));
              dir.normalize();

              dir *= length.val(comp_data);

              std::optional<Eigen::Affine2d> translation_point =
                  point.positionObject(comp_data).getCurrentPosition(comp_data.wm, "", comp_data.time);

              if (!translation_point.has_value()) return {""};

              return {"", translation_point->translation().x() + dir.x(),
                      translation_point->translation().y() + dir.y()};
          }},
      cutoff_start(std::move(cutoff_start)),
      cutoff_end(std::move(cutoff_end)){};

VectorWithVelocityStamped LineShape::getTransformToClosestPoint(const ComponentData& comp_data,
                                                                const transform::Position& robot_position,
                                                                const transform::Position& observe_position) const {
    VectorWithVelocityStamped trans_and_vel;
    trans_and_vel.stamp = comp_data.time;

    // translation
    std::optional<Eigen::Affine2d> translation_to_start_in_obs_frame =
        this->start.positionObject(comp_data).getCurrentPosition(comp_data.wm, observe_position, comp_data.time);
    std::optional<Eigen::Affine2d> translation_to_end_in_obs_frame =
        this->end.positionObject(comp_data).getCurrentPosition(comp_data.wm, observe_position, comp_data.time);

    std::optional<Eigen::Affine2d> translation_to_ref_in_obs_frame =
        robot_position.getCurrentPosition(comp_data.wm, observe_position, comp_data.time);

    double lambda = 0.0;
    if (translation_to_start_in_obs_frame.has_value() && translation_to_end_in_obs_frame.has_value() &&
        translation_to_ref_in_obs_frame.has_value()) {
        Eigen::Vector2d v_line_start = translation_to_start_in_obs_frame->translation();
        double line_start_x = v_line_start.x();
        double line_start_y = v_line_start.y();

        Eigen::Vector2d v_line = translation_to_end_in_obs_frame->translation() - v_line_start;
        double line_x = v_line.x();
        double line_y = v_line.y();

        Eigen::Vector2d v_point = translation_to_ref_in_obs_frame->translation();
        double p_x = v_point.x();
        double p_y = v_point.y();

        if (line_x == 0.0) {
            if (line_y * line_y == 0.0) {
                lambda = 0.0;
            } else {
                lambda = (p_y - line_start_y) / line_y;
            }
        } else if (line_y == 0.0) {
            if (line_x * line_x == 0.0) {
                lambda = 0.0;
            } else {
                lambda = (p_x - line_start_x) / line_x;
            }
        } else {
            if (line_x * line_x + line_y * line_y == 0.0) {
                lambda = 0.0;
            } else {
                lambda = (line_x * (p_x - line_start_x) + line_y * (p_y - line_start_y)) /
                         (line_x * line_x + line_y * line_y);
            }
        }

        double lambda_start = 0.0;
        double lambda_end = 0.0;
        if (v_line.norm() > 0) {
            lambda_start = cutoff_start.val(comp_data) / v_line.norm();
            lambda_end = 1.0 - cutoff_end.val(comp_data) / v_line.norm();
        }

        if (lambda > lambda_end) lambda = lambda_end;
        if (lambda < lambda_start) lambda = lambda_start;

        Eigen::Vector2d v_lot_point = v_line_start + lambda * v_line;
        trans_and_vel.vec = v_lot_point - v_point;
    }

    // velocity
    std::optional<Eigen::Vector3d> velocity1 = this->start.positionObject(comp_data).getVelocity(
        comp_data.wm, robot_position.getFrame(), observe_position.getFrame(), comp_data.time);
    std::optional<Eigen::Vector3d> velocity2 = this->end.positionObject(comp_data).getVelocity(
        comp_data.wm, robot_position.getFrame(), observe_position.getFrame(), comp_data.time);
    if (velocity1 && velocity2) {
        trans_and_vel.velocity = velocity1->head(2) + ((velocity2->head(2) - velocity1->head(2)) * lambda);
    }

    // visualization
    marker::Line l(comp_data.wm->getGlobalFrame());

    std::optional<Eigen::Affine2d> l_start = start.positionObject(comp_data).getCurrentPosition(comp_data.wm);
    std::optional<Eigen::Affine2d> l_end = end.positionObject(comp_data).getCurrentPosition(comp_data.wm);

    if (l_start.has_value() && l_end.has_value()) {
        const Eigen::Vector2d v_start = l_start->translation();
        const Eigen::Vector2d v_end = l_end->translation();
        const Eigen::Vector2d v_line = v_end - v_start;
        double lambda_start = 0.0;
        double lambda_end = 0.0;
        if (v_line.norm() > 0) {
            lambda_start = cutoff_start.val(comp_data) / v_line.norm();
            lambda_end = 1.0 - cutoff_end.val(comp_data) / v_line.norm();
        }

        l.setLinePoints({(v_start + lambda_start * v_line).x(), (v_start + lambda_start * v_line).y()},
                        {(v_start + lambda_end * v_line).x(), (v_start + lambda_end * v_line).y()});
    }
    comp_data.ma.displayMarker(l);

    return trans_and_vel;
}

std::optional<Eigen::Vector2d> LineShape::getCenter(const ComponentData& comp_data,
                                                    const transform::Position& observe_position) const {
    std::optional<Eigen::Affine2d> l_start =
        start.positionObject(comp_data).getCurrentPosition(comp_data.wm, observe_position, comp_data.time);
    std::optional<Eigen::Affine2d> l_end =
        end.positionObject(comp_data).getCurrentPosition(comp_data.wm, observe_position, comp_data.time);

    if (l_start.has_value() && l_end.has_value()) {
        const Eigen::Vector2d v_start = l_start->translation();
        const Eigen::Vector2d v_end = l_end->translation();
        const Eigen::Vector2d v_line = v_end - v_start;
        double lambda_start = 0.0;
        double lambda_end = 0.0;
        if (v_line.norm() > 0) {
            lambda_start = cutoff_start.val(comp_data) / v_line.norm();
            lambda_end = 1.0 - cutoff_end.val(comp_data) / v_line.norm();
        }

        return v_start + (lambda_start + lambda_end) / 2 * v_line;
    }
    return std::nullopt;
}

}  // namespace luhsoccer::robot_control
