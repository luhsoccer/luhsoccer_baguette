
#pragma once
#define CHANGE_DRIBBLER_STEP

#include <utility>

#include "local_planner/skills/skill_util.hpp"
#include "local_planner/skills/abstract_step.hpp"
#include "robot_interface/robot_interface_types.hpp"

namespace luhsoccer::local_planner {

class ConditionStep : public AbstractStep {
   public:
    [[nodiscard]] std::pair<StepState, robot_interface::RobotCommand> calcCommandMessage(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const std::shared_ptr<AvoidanceManager>& am, const time::TimePoint time = time::TimePoint(0)) const override;

    [[nodiscard]] std::vector<Marker> getVisualizationMarkers(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const time::TimePoint time = time::TimePoint(0)) const override;

    template <typename T>
    void addIfStep(T&& step) {
        static_assert(std::is_base_of<AbstractStep, T>::value);
        this->if_steps.push_back(std::make_shared<T>(std::forward<T>(step)));
    }

    template <typename T>
    void addElseStep(T&& step) {
        static_assert(std::is_base_of<AbstractStep, T>::value);
        this->else_steps.push_back(std::make_shared<T>(std::forward<T>(step)));
    }

    ConditionStep(const BoolComponentParam condition);

   private:
    BoolComponentParam condition;
    std::vector<std::shared_ptr<const AbstractStep>> if_steps{};
    std::vector<std::shared_ptr<const AbstractStep>> else_steps{};
};

}  // namespace luhsoccer::local_planner