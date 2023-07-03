#pragma once

#include "local_planner_components/steps/drive_step_constraint.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::local_planner {

class BallPlacement : public DriveStepConstraint {
   public:
    BallPlacement();

   private:
    [[nodiscard]] bool conditionMeetImpl(const std::shared_ptr<const transform::WorldModel>& wm,
                                         const TaskData& td) const override;
    [[nodiscard]] std::vector<std::shared_ptr<const AbstractTargetFeature>> getAdditionalTargetFeaturesImpl()
        const override;
    [[nodiscard]] std::vector<std::shared_ptr<const AbstractCFObstacle>> getAdditionalObstacleFeaturesImpl()
        const override;
    DoubleComponentParam weight;
    static const inline std::string BALL_PLACEMENT_POSITION_FRAME = "ball_placement_position";
};
}  // namespace luhsoccer::local_planner