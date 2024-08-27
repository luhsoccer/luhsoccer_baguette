#pragma once

#include <utility>

#include "robot_control/components/abstract_step.hpp"
#include "robot_control/components/component_util.hpp"
#include "robot_interface/robot_interface_types.hpp"

namespace luhsoccer::robot_control {

class DribblerStep : public AbstractStep {
   public:
    [[nodiscard]] std::pair<StepState, robot_interface::RobotCommand> calcCommandMessage(
        const ComponentData& comp_data) const override;

    explicit DribblerStep(robot_interface::DribblerMode mode);

   private:
    robot_interface::DribblerMode mode;
};

}  // namespace luhsoccer::robot_control