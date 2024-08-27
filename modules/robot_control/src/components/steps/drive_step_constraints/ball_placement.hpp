#pragma once

#include "../drive_step_constraint.hpp"

namespace luhsoccer::robot_control {

class BallPlacementConstraint : public DriveStepConstraint {
   public:
    BallPlacementConstraint() : DriveStepConstraint() { this->initFeatures(); };

   private:
    [[nodiscard]] bool conditionMeetImpl(const ComponentData& comp_data) const override;

    [[nodiscard]] std::vector<std::shared_ptr<const AbstractObstacle>> getAdditionalObstacleFeaturesImpl()
        const override;

    static const inline std::string BALL_PLACEMENT_POSITION_FRAME = "ball_placement_position";
};
}  // namespace luhsoccer::robot_control