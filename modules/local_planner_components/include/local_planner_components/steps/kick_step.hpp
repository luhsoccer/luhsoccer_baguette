#pragma once

#include <utility>

#include "local_planner/skills/skill_util.hpp"
#include "local_planner/skills/abstract_step.hpp"
#include "robot_interface/robot_interface_types.hpp"

namespace luhsoccer::local_planner {

class KickStep : public AbstractStep {
   public:
    KickStep(DoubleComponentParam velocity, BoolComponentParam wait, robot_interface::KickExecuteTime execute_time)
        : velocity(std::move(velocity)), wait(std::move(wait)), execute_time(execute_time){};

    [[nodiscard]] std::pair<StepState, robot_interface::RobotCommand> calcCommandMessage(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const std::shared_ptr<AvoidanceManager>& am, time::TimePoint time = time::TimePoint(0)) const override;

    // [[nodiscard]] std::vector<Marker> getVisualizationMarkers(
    //     const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
    //     const time::TimePoint time = time::TimePoint(0)) const override;

   private:
    class HadBallFlagT {};
    DoubleComponentParam velocity;
    BoolComponentParam wait;
    robot_interface::KickExecuteTime execute_time;
};
}  // namespace luhsoccer::local_planner