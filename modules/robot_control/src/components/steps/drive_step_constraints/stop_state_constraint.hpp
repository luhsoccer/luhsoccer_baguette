#pragma once

#include "../drive_step_constraint.hpp"

namespace luhsoccer::robot_control {

class StopStateConstraint : public DriveStepConstraint {
   public:
    StopStateConstraint() : DriveStepConstraint() { this->initFeatures(); };

   private:
    [[nodiscard]] bool conditionMeetImpl(const ComponentData& comp_data) const override;

    [[nodiscard]] std::vector<std::shared_ptr<const AbstractObstacle>> getAdditionalObstacleFeaturesImpl()
        const override;
};
}  // namespace luhsoccer::robot_control