#pragma once

#include "robot_control/components/abstract_rotation_control.hpp"

namespace luhsoccer::robot_control {

class RotateToMoveDirectionControl : public AbstractRotationControl {
   public:
    RotateToMoveDirectionControl(BoolComponentParam reverse_direction = false,
                                 const std::optional<DoubleComponentParam>& k_p = std::nullopt,
                                 const std::optional<DoubleComponentParam>& max_vel_theta = std::nullopt)
        : AbstractRotationControl(max_vel_theta),
          k_p(k_p.has_value() ? k_p.value() : robotControlConfig().rotation_k_p),
          reverse_direction(std::move(reverse_direction)){};

    [[nodiscard]] std::pair<double, bool> calcRotationVelocity(
        const ComponentData& comp_data, const transform::Position& robot_position) const override;

   private:
    DoubleComponentParam k_p;
    BoolComponentParam reverse_direction;
};

}  // namespace luhsoccer::robot_control