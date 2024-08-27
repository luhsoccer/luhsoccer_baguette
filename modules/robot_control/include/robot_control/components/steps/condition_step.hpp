#pragma once

#include "robot_control/components/abstract_step.hpp"
#include "robot_control/components/component_util.hpp"
#include "robot_interface/robot_interface_types.hpp"

namespace luhsoccer::robot_control {

class ConditionStep : public AbstractStep {
   public:
    [[nodiscard]] std::pair<StepState, robot_interface::RobotCommand> calcCommandMessage(
        const ComponentData& comp_data) const override;

    template <typename T>
    void addIfStep(T&& step) {
        static_assert(std::is_base_of<AbstractStep, T>::value);
        this->if_steps.push_back(std::make_shared<T>(std::forward<T>(step)));
    }

    void addIfStepPtr(const std::shared_ptr<const robot_control::AbstractStep>& s) { this->if_steps.push_back(s); }

    template <typename T>
    void addElseStep(T&& step) {
        static_assert(std::is_base_of<AbstractStep, T>::value);
        this->else_steps.push_back(std::make_shared<T>(std::forward<T>(step)));
    }

    void addElseStepPtr(const std::shared_ptr<const robot_control::AbstractStep>& s) { this->else_steps.push_back(s); }

    explicit ConditionStep(const BoolComponentParam condition);

   private:
    BoolComponentParam condition;
    std::vector<std::shared_ptr<const AbstractStep>> if_steps{};
    std::vector<std::shared_ptr<const AbstractStep>> else_steps{};
};

}  // namespace luhsoccer::robot_control