
#pragma once
#define CHANGE_DRIBBLER_STEP

#include <utility>

#include "local_planner/skills/skill_util.hpp"
#include "local_planner/skills/abstract_step.hpp"
#include "robot_interface/robot_interface_types.hpp"

namespace luhsoccer::local_planner {

class DribblerStep : public AbstractStep {
   public:
    [[nodiscard]] std::pair<StepState, robot_interface::RobotCommand> calcCommandMessage(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const std::shared_ptr<AvoidanceManager>& am, const time::TimePoint time = time::TimePoint(0)) const override;

    DribblerStep(robot_interface::DribblerMode mode);

   private:
    robot_interface::DribblerMode mode;
};

}  // namespace luhsoccer::local_planner