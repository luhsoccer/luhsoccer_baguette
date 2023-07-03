#pragma once

#include "local_planner/skills/skill_util.hpp"
#include "local_planner_components/steps/drive_step.hpp"

namespace luhsoccer::local_planner {

class TurnAroundBallStep : public DriveStep {
   public:
    TurnAroundBallStep(ComponentPosition align_position);

   private:
    /*Store Component parameters here*/
};

}  // namespace luhsoccer::local_planner