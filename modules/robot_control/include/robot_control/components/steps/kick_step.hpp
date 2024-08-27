#pragma once

#include <utility>

#include "robot_control/components/abstract_step.hpp"
#include "robot_control/components/component_util.hpp"
#include "robot_interface/robot_interface_types.hpp"

namespace luhsoccer::robot_control {
constexpr double MAX_KICK_VELOCITY = 6.5;
class KickStep : public AbstractStep {
   public:
    KickStep(DoubleComponentParam velocity, BoolComponentParam wait, robot_interface::KickExecuteTime execute_time,
             BoolComponentParam chip = false)
        : velocity(std::move(velocity)), wait(std::move(wait)), chip(std::move(chip)), execute_time(execute_time){};

    [[nodiscard]] std::pair<StepState, robot_interface::RobotCommand> calcCommandMessage(
        const ComponentData& comp_data) const override;

   private:
    class HadBallFlagT {};
    DoubleComponentParam velocity;
    BoolComponentParam wait;
    BoolComponentParam chip;
    robot_interface::KickExecuteTime execute_time;
};

class ChangeKickerModeStep : public AbstractStep {
   public:
    ChangeKickerModeStep(BoolComponentParam chip) : chip(std::move(chip)){};

    [[nodiscard]] std::pair<StepState, robot_interface::RobotCommand> calcCommandMessage(
        const ComponentData& comp_data) const override;

   private:
    BoolComponentParam chip;
};
}  // namespace luhsoccer::robot_control