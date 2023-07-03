

#pragma once

#include <utility>
#include <variant>
#include "local_planner/skills/abstract_rotation_control.hpp"
namespace luhsoccer::local_planner {

class RotateToMoveDirectionControl : public AbstractRotationControl {
   public:
    RotateToMoveDirectionControl(BoolComponentParam reverse_direction = false,
                                 const std::optional<DoubleComponentParam>& rotation_k_g = std::nullopt,
                                 const std::optional<DoubleComponentParam>& rotation_k_v = std::nullopt,
                                 const std::optional<DoubleComponentParam>& max_vel_theta = std::nullopt)
        : AbstractRotationControl(max_vel_theta),
          rotation_k_g(localPlannerConfig().rotation_k_g),
          rotation_k_v(localPlannerConfig().rotation_k_v),
          reverse_direction(std::move(reverse_direction)) {
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
    DoubleComponentParam rotation_k_g;
    DoubleComponentParam rotation_k_v;
    BoolComponentParam reverse_direction;
};

}  // namespace luhsoccer::local_planner