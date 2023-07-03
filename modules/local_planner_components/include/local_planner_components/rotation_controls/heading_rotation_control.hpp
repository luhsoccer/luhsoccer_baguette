

#pragma once

#include <utility>
#include <variant>
#include "local_planner/skills/abstract_rotation_control.hpp"
namespace luhsoccer::local_planner {

constexpr double THREE_DEGREE_IN_RADIAN = 0.052;

class HeadingRotationControl : public AbstractRotationControl {
   public:
    HeadingRotationControl(const DoubleComponentParam& angle,
                           const ComponentPosition& observe_position = ComponentPosition(""),
                           DoubleComponentParam heading_tolerance = THREE_DEGREE_IN_RADIAN,
                           BoolComponentParam velocity_zero_for_reach = false,
                           const std::optional<DoubleComponentParam>& rotation_k_g = std::nullopt,
                           const std::optional<DoubleComponentParam>& rotation_k_v = std::nullopt,
                           const std::optional<DoubleComponentParam>& max_vel_theta = std::nullopt)
        : AbstractRotationControl(max_vel_theta),
          angle_rotation_target({angle, observe_position}),
          heading_tolerance(std::move(heading_tolerance)),
          rotation_k_g(localPlannerConfig().rotation_k_g),
          rotation_k_v(localPlannerConfig().rotation_k_v),
          velocity_zero_for_reach(std::move(velocity_zero_for_reach)) {
        if (rotation_k_g) this->rotation_k_g = *rotation_k_g;
        if (rotation_k_v) this->rotation_k_v = *rotation_k_v;
    };
    HeadingRotationControl(const ComponentPosition& align_position, const BoolComponentParam& face_away = false,
                           DoubleComponentParam heading_tolerance = THREE_DEGREE_IN_RADIAN,
                           BoolComponentParam velocity_zero_for_reach = false,
                           const std::optional<DoubleComponentParam>& rotation_k_g = std::nullopt,
                           const std::optional<DoubleComponentParam>& rotation_k_v = std::nullopt,
                           const std::optional<DoubleComponentParam>& max_vel_theta = std::nullopt)
        : AbstractRotationControl(max_vel_theta),
          position_rotation_target({align_position, face_away}),
          heading_tolerance(std::move(heading_tolerance)),
          rotation_k_g(localPlannerConfig().rotation_k_g),
          rotation_k_v(localPlannerConfig().rotation_k_v),
          velocity_zero_for_reach(std::move(velocity_zero_for_reach)) {
        if (rotation_k_g) this->rotation_k_g = *rotation_k_g;
        if (rotation_k_v) this->rotation_k_v = *rotation_k_v;
    };

    [[nodiscard]] std::pair<double, bool> calcRotationForce(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const time::TimePoint& time = time::TimePoint(0)) const override;

    [[nodiscard]] std::vector<Marker> getVisualizationMarker(const std::shared_ptr<const transform::WorldModel>& wm,
                                                             const TaskData& td, const RobotIdentifier& robot,
                                                             time::TimePoint time = time::TimePoint(0)) const override;

   private:
    static constexpr double ZERO_VELOCITY_TOLERANCE = 0.1;

    std::optional<std::pair<DoubleComponentParam, ComponentPosition>> angle_rotation_target;
    std::optional<std::pair<ComponentPosition, BoolComponentParam>> position_rotation_target;
    DoubleComponentParam heading_tolerance;
    DoubleComponentParam rotation_k_g;
    DoubleComponentParam rotation_k_v;
    BoolComponentParam velocity_zero_for_reach;
};

}  // namespace luhsoccer::local_planner