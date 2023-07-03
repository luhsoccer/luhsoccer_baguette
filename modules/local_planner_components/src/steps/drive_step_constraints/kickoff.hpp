#pragma once

#include "local_planner_components/steps/drive_step_constraint.hpp"

namespace luhsoccer::local_planner {

class Kickoff : public DriveStepConstraint {
   public:
    Kickoff(bool enemy) : DriveStepConstraint(), enemy(enemy) { this->initFeatures(); }

   private:
    [[nodiscard]] bool conditionMeetImpl(const std::shared_ptr<const transform::WorldModel>& wm,
                                         const TaskData& /*td*/) const override;
    [[nodiscard]] std::vector<std::shared_ptr<const AbstractCFObstacle>> getAdditionalObstacleFeaturesImpl()
        const override;
    bool enemy;
};
}  // namespace luhsoccer::local_planner
