#pragma once

#include "experiment_logger/experiment_logger.hpp"
#include "scenario/scenario.hpp"
#include "module.hpp"

namespace luhsoccer {

namespace robot_interface {
class RobotInterface;
}
namespace scenario {
class ScenarioExecutor : public BaguetteModule {
   public:
    virtual ~ScenarioExecutor() = default;
    ScenarioExecutor(const ScenarioExecutor&) = delete;
    ScenarioExecutor(ScenarioExecutor&&) = delete;
    ScenarioExecutor& operator=(const ScenarioExecutor&) = delete;
    ScenarioExecutor& operator=(ScenarioExecutor&&) = delete;
    ScenarioExecutor(std::shared_ptr<const transform::WorldModel> wm,
                     simulation_interface::SimulationInterface& simulation_interface,
                     local_planner::LocalPlannerModule& local_planner_module, marker::MarkerService& ms,
                     const skills::BodSkillBook& skill_book, const robot_interface::RobotInterface& robot_interface);
    constexpr std::string_view moduleName() override { return "ScenarioExecutor"; }

    void loop(std::atomic_bool& should_run) override;

    bool startScenario(const Scenario& scenario, size_t repetitions = 1);

    void cancelScenario();

   private:
    void executeScenario(std::atomic_bool& should_run, Scenario& scenario);

    const static Scenario& scenario;
    std::shared_ptr<const transform::WorldModel> wm;
    simulation_interface::SimulationInterface& simulation_interface;
    local_planner::LocalPlannerModule& local_planner_module;
    const skills::BodSkillBook& skill_book;
    const robot_interface::RobotInterface& robot_interface;

    enum class State { IDLE, SCENARIO_REQUESTED, RUNNING };
    std::atomic<State> state;
    std::optional<Scenario> current_scenario;
    size_t repetitions{1};

    logger::Logger logger{"ScenarioExecutor"};

    experiment_logger::ExperimentLogger exp_logger;
};

}  // namespace scenario
}  // namespace luhsoccer