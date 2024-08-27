#include <utility>

#include "scenario/scenario_executor.hpp"
#include "scenario/scenario_book.hpp"
#include "robot_interface/robot_interface.hpp"
#include "robot_control/robot_control_module.hpp"
namespace luhsoccer::scenario {

// #define STATIC_MODE
// const Scenario& ScenarioExecutor::scenario = book::SWITCH;
// const std::string STRATEGY_NAME = "coop-";

ScenarioExecutor::ScenarioExecutor(std::shared_ptr<const transform::WorldModel> wm,
                                   simulation_interface::SimulationInterface& simulation_interface,
                                   robot_control::RobotControlModule& robot_control_module, marker::MarkerService& ms,
                                   const skills::SkillLibrary& skill_lib,
                                   const robot_interface::RobotInterface& robot_interface)
    : wm(std::move(wm)),
      simulation_interface(simulation_interface),
      robot_control_module(robot_control_module),
      skill_library(skill_lib),
      robot_interface(robot_interface),
      state(State::IDLE),
      exp_logger(ms, "experiment_logs") {}

bool ScenarioExecutor::startScenario(const Scenario& scenario, size_t repetitions) {
    if (this->state != State::IDLE) cancelScenario();
    this->current_scenario = scenario;
    this->state = State::SCENARIO_REQUESTED;
    this->repetitions = repetitions;
    return true;
}

void ScenarioExecutor::cancelScenario() {
    if (this->state != State::RUNNING) return;
    this->state = State::IDLE;
    if (!this->current_scenario.has_value()) return;
    this->current_scenario->stop(this->robot_control_module);
}

void ScenarioExecutor::loop(std::atomic_bool& should_run) {
#ifndef STATIC_MODE
    time::Rate rate(2);
    while (should_run && this->state == State::IDLE) rate.sleep();
#else
    std::this_thread::sleep_for(time::Duration(3.0));
    this->current_scenario = ScenarioExecutor::scenario;
    this->state = State::SCENARIO_REQUESTED;
#endif
    if (this->state == State::SCENARIO_REQUESTED && this->current_scenario.has_value()) {
        this->executeScenario(should_run, this->current_scenario.value());
        this->state = State::IDLE;
    }
}

void ScenarioExecutor::executeScenario(std::atomic_bool& should_run, Scenario& scenario) {
    for (size_t i = 0; i < this->repetitions; i++) {
        bool real_live_mode = this->robot_interface.getConnectionType() != robot_interface::RobotConnection::SIMULATION;
        this->state = State::SCENARIO_REQUESTED;
        this->logger.info("Setting up scenario '{}'...", scenario.getName());
        if (!scenario.setup(this->wm, this->simulation_interface, this->robot_control_module, this->skill_library,
                            real_live_mode))
            return;
        this->state = State::RUNNING;
        std::this_thread::sleep_for(time::Duration(0.3));

        this->logger.info("Starting scenario '{}'...", scenario.getName());

        // start experiment logger
        std::vector<experiment_logger::TrackTarget> targets;
        for (const auto& robot : scenario.getInvolvedAllyRobots()) {
            experiment_logger::TrackTarget target{fmt::format("{}", robot), transform::Position(robot.getFrame()), "",
                                                  ""};
            targets.push_back(target);
        }

        std::vector<transform::Position> obstacles;
        for (const auto& robot : scenario.getInvolvedEnemyRobots()) {
            obstacles.emplace_back(robot.getFrame());
        }

        this->exp_logger.startExperiment(scenario.getName(), scenario.getName(), this->wm, targets, obstacles);
        time::TimePoint start_time = time::now();
        if (!scenario.execute(this->robot_control_module, this->skill_library)) return;

        time::Rate rate(100);
        while (should_run && !scenario.isFinished(this->robot_control_module, time::now() - start_time)) {
            this->exp_logger.newDataPoint();
            scenario.teleportRobots(wm, this->simulation_interface, this->robot_control_module, this->skill_library,
                                    real_live_mode, time::now() - start_time);
            rate.sleep();
        }
        if (!this->exp_logger.endExperiment()) this->logger.warning("experiment data could not be saved!");
        this->logger.info("Finished scenario '{}'.", scenario.getName());
    }
}

}  // namespace luhsoccer::scenario