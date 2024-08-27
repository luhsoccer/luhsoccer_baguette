#pragma once

#include "../drive_step_constraint.hpp"
#include "robot_control/components/component_util.hpp"

namespace luhsoccer::robot_control {

class DefenseAreaConstraint : public DriveStepConstraint {
   public:
    DefenseAreaConstraint() : DriveStepConstraint() { this->initFeatures(); };

   private:
    [[nodiscard]] bool conditionMeetImpl(const ComponentData& comp_data) const override;
    // [[nodiscard]] std::vector<std::shared_ptr<const AbstractTargetFeature>> getAdditionalTargetFeaturesImpl()
    //     const override;
    [[nodiscard]] std::vector<std::shared_ptr<const AbstractObstacle>> getAdditionalObstacleFeaturesImpl()
        const override;
    static constexpr double FIELD_RUNOFF_WIDTH = 0.3;
};
}  // namespace luhsoccer::robot_control