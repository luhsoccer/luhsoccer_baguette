#pragma once

#include "robot_control/components/steps/drive_step.hpp"

namespace luhsoccer::robot_control {

class TurnAroundBallStep : public DriveStep {
   public:
    explicit TurnAroundBallStep(ComponentPosition align_position);

   private:
    /*Store Component parameters here*/
};

}  // namespace luhsoccer::robot_control