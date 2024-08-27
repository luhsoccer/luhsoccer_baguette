#pragma once
#include "robot_control/components/abstract_rotation_control.hpp"

namespace luhsoccer::robot_control {

constexpr double THREE_DEGREE_IN_RADIAN = 0.052;

class HeadingRotationControl : public AbstractRotationControl {
   public:
    explicit HeadingRotationControl(const DoubleComponentParam& angle,
                                    const ComponentPosition& observe_position = ComponentPosition(""),
                                    DoubleComponentParam heading_tolerance = THREE_DEGREE_IN_RADIAN,
                                    BoolComponentParam velocity_zero_for_reach = false,
                                    const std::optional<DoubleComponentParam>& k_p = std::nullopt,
                                    const std::optional<DoubleComponentParam>& max_vel_theta = std::nullopt);

    explicit HeadingRotationControl(const ComponentPosition& align_position,
                                    const BoolComponentParam& face_away = false,
                                    DoubleComponentParam heading_tolerance = THREE_DEGREE_IN_RADIAN,
                                    BoolComponentParam velocity_zero_for_reach = false,
                                    const std::optional<DoubleComponentParam>& k_p = std::nullopt,
                                    const std::optional<DoubleComponentParam>& max_vel_theta = std::nullopt);

    [[nodiscard]] std::pair<double, bool> calcRotationVelocity(
        const ComponentData& comp_data, const transform::Position& robot_position) const override;

   private:
    static constexpr double ZERO_VELOCITY_TOLERANCE = 0.1;

    std::optional<std::pair<DoubleComponentParam, ComponentPosition>> angle_rotation_target{std::nullopt};
    std::optional<std::pair<ComponentPosition, BoolComponentParam>> position_rotation_target{std::nullopt};
    DoubleComponentParam heading_tolerance;
    DoubleComponentParam rotation_k_p;
    BoolComponentParam velocity_zero_for_reach;
};

}  // namespace luhsoccer::robot_control