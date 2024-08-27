#pragma once

#include "robot_control/simulation_types.hpp"
#include "robot_dynamic.hpp"
#include "robot_interface/robot_interface_types.hpp"
#include "robot_controller.hpp"
#include "cooperation_module.hpp"

namespace luhsoccer::robot_control {

namespace luhsoccer::transform {
class WorldModel;
}

class Simulator {
   public:
    Simulator(unsigned long id, const std::shared_ptr<const transform::WorldModel>& base_wm,
              const std::vector<SimulationTask>& tasks, const time::TimePoint& start_time);

    SimulationResult runSimulation(const std::optional<time::TimePoint>& start_from = std::nullopt);

    void stopSimulation() { this->should_run = false; }

   private:
    std::unordered_map<RobotIdentifier, robot_interface::RobotCommand> calcCommands();

    void applyRobotCommands(const std::unordered_map<RobotIdentifier, robot_interface::RobotCommand>& commands);

    bool moveFrame(const std::string& frame, const time::Duration& step_duration,
                   const Eigen::Vector3d& acceleration = {0.0, 0.0, 0.0});

    bool applyRobotCommandVelocity(RobotIdentifier robot, const time::Duration& step_duration,
                                   const Eigen::Vector3d& local_command_velocity);

    static Eigen::Vector3d getRobotCommandVelocityFromCommand(const robot_interface::RobotCommand& command);

    // configuration
    unsigned long id;
    std::shared_ptr<const transform::WorldModel> base_wm;
    std::shared_ptr<transform::WorldModel> wm;
    std::unordered_map<RobotIdentifier, SimulationTask> tasks;
    RobotDynamic robot_dynamic;
    double simulation_frequency;
    std::atomic_bool should_run{true};
    time::TimePoint start_time;

    // runtime variables
    MarkerAdapter ma;
    enum class SimulatorState { CREATED, RUNNING, FINISHED };
    std::unordered_map<RobotIdentifier, RobotController> robot_controllers;
    CooperationModule coop_module;
    std::atomic<SimulatorState> state{SimulatorState::CREATED};
    time::TimePoint current_time;
    std::map<RobotIdentifier, time::TimePoint> inactive_commands_since;

    logger::Logger logger{"RobotControlSimulator"};
};

}  // namespace luhsoccer::robot_control
