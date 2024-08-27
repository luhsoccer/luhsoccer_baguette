#pragma once

#include "../drive_step_constraint.hpp"

namespace luhsoccer::robot_control {
class AbstractObstacle;
class LeaveFieldConstraint : public DriveStepConstraint {
   public:
    LeaveFieldConstraint() : DriveStepConstraint() {
        this->initFeatures();
        this->setActive(false);
    }

   private:
    [[nodiscard]] bool conditionMeetImpl(const ComponentData&) const override { return true; }
    [[nodiscard]] std::vector<std::shared_ptr<const AbstractObstacle>> getAdditionalObstacleFeaturesImpl()
        const override;
};
}  // namespace luhsoccer::robot_control