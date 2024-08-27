#include "simulator.hpp"
#include "config/robot_control_config.hpp"
#include "core/visit.hpp"
#include "robot_control/components/component_util.hpp"

namespace luhsoccer::robot_control {
Simulator::Simulator(unsigned long id, const std::shared_ptr<const transform::WorldModel>& base_wm,
                     const std::vector<SimulationTask>& tasks, const time::TimePoint& start_time)
    : id(id),
      base_wm(base_wm),
      wm(std::make_shared<transform::WorldModel>(*base_wm)),
      simulation_frequency(robotControlConfig().simulation_frequency),
      start_time(start_time.asSec() == 0.0 ? time::now() : start_time),
      ma(),
      coop_module() {
    for (const auto& task : tasks) {
        this->tasks.emplace(task.task_data.robot, task);
        auto [it, success] = this->robot_controllers.try_emplace(task.task_data.robot, this->wm, this->ma,
                                                                 task.task_data.robot, this->coop_module);
        if (success) {
            it->second.startSimulation();
            it->second.setTask(task.skill, task.task_data);
        }
    }
}

bool isCommandActive(const robot_interface::RobotCommand& cmd) {
    constexpr double SMALL_VALUE = 0.001;
    if (cmd.kick_command.has_value() && cmd.kick_command->velocity > SMALL_VALUE) return true;
    if (cmd.move_command.has_value()) {
        auto visitor = overload{
            [](const robot_interface::RobotVelocityControl& vel_control) -> bool {
                return vel_control.desired_velocity.norm() > SMALL_VALUE;
            },
            [](const robot_interface::RobotPositionControl& pos_control) -> bool {
                return pos_control.desired_velocity.norm() > SMALL_VALUE;
            },
        };
        if (std::visit(visitor, cmd.move_command.value())) return true;
    }
    return false;
}

SimulationResult Simulator::runSimulation(const std::optional<time::TimePoint>& start_from) {
    this->current_time = start_from.value_or(this->start_time);

    if (this->tasks.empty()) {
        this->logger.error("Tasks of Simulator with id {:d} is empty, when trying to start simulation.", this->id);
        return {};
    }
    if (this->state == SimulatorState::RUNNING) return {};
    bool run = true;
    // if (!this->start_time.has_value()) this->start_time = time::now();
    // time::TimePoint current_time = this->start_time.value();
    constexpr time::Duration TIMEOUT = 30.0;
    while (run && this->should_run) {
        auto commands = this->calcCommands();
        for (const auto& cmd : commands) {
            if (!isCommandActive(cmd.second)) {
                if (this->inactive_commands_since.count(cmd.first) == 0) {
                    this->inactive_commands_since[cmd.first] = this->current_time;
                }
            } else {
                if (this->inactive_commands_since.count(cmd.first)) {
                    this->inactive_commands_since.erase(cmd.first);
                }
            }
        }
        this->applyRobotCommands(commands);

        bool controller_running = false;
        for (auto& [robot, controllers] : this->robot_controllers) {
            if (controllers.getState() == RobotControllerState::RUNNING) {
                // agent is not truely running if commands are inactive for one second
                if (this->inactive_commands_since.count(robot)) {
                    constexpr time::Duration INACTIVE_TIMEOUT = 1.0;
                    if (this->inactive_commands_since[robot] < this->current_time - INACTIVE_TIMEOUT) {
                        continue;
                    }
                }
                controller_running = true;
                break;
            }
        }
        bool timeout = (this->current_time - this->start_time) > TIMEOUT;
        run = controller_running && !timeout;
        this->current_time += time::Duration(1.0 / this->simulation_frequency);
    }

    if (!this->should_run) {
        this->logger.info("Simulation {:d} received stop command.", this->id);
        return {};
    }
    // for (auto& [robot, agent] : this->agents) {
    //     if (agent.getCurrentState() != Agent::AgentState::FINISHED) {
    //         this->warning.info("One or more agents of simulation {} did not exit with state FINISHED!",
    //                     this->simulation_id);
    //         // return;
    //     }
    // }
    // calc result
    // update task_data in tasks from agents
    for (auto& task : this->tasks) {
        auto controller_it = this->robot_controllers.find(task.first);
        if (controller_it != this->robot_controllers.end()) {
            auto td = controller_it->second.getCurrentTaskData();
            if (td.has_value()) task.second.task_data = td.value();
        };
    }
    SimulationResult result{this->wm, this->tasks, this->start_time, current_time};
    return result;
}

std::unordered_map<RobotIdentifier, robot_interface::RobotCommand> Simulator::calcCommands() {
    std::unordered_map<RobotIdentifier, robot_interface::RobotCommand> commands;
    for (auto& [robot, controller] : this->robot_controllers) {
        auto [cmd, state] = controller.calcCurrentCommandMessage();
        if (cmd.has_value()) commands[robot] = cmd.value();
    }
    return commands;
}

void Simulator::applyRobotCommands(const std::unordered_map<RobotIdentifier, robot_interface::RobotCommand>& commands) {
    // move all robots
    for (const auto& robot : this->wm->getVisibleRobots()) {
        // check if command exists for this robot
        if (robot.isAlly() && commands.find(robot) != commands.end()) {
            auto robot_command_vel = this->getRobotCommandVelocityFromCommand(commands.at(robot));
            this->applyRobotCommandVelocity(robot, {1.0 / this->simulation_frequency}, robot_command_vel);
        } else {
            this->moveFrame(robot.getFrame(), {1.0 / this->simulation_frequency});
        }
    }
    // move ball
    this->moveFrame(this->wm->getBallFrame(), {1.0 / this->simulation_frequency});
}

bool Simulator::moveFrame(const std::string& frame, const time::Duration& step_duration,
                          const Eigen::Vector3d& acceleration) {
    auto current_pos = this->wm->getTransform(frame, "");
    auto current_vel = this->wm->getVelocity(frame, "", "");

    if (!current_pos.has_value() || !current_vel.has_value()) return false;

    Eigen::Vector3d new_vel = current_vel->velocity + acceleration * step_duration.asSec();
    Eigen::Affine2d delta_pos = Eigen::Translation2d(new_vel.head(2) * step_duration.asSec()) *
                                Eigen::Rotation2Dd(new_vel[2] * step_duration.asSec());
    Eigen::Affine2d new_pos = current_pos->transform * delta_pos;

    transform::TransformWithVelocity trans_with_vel;
    trans_with_vel.header.child_frame = frame;
    trans_with_vel.header.stamp = current_time + step_duration;
    trans_with_vel.header.parent_frame = "";
    trans_with_vel.transform = new_pos;
    trans_with_vel.velocity = new_vel;
    this->wm->pushTransform(trans_with_vel);
    return true;
}

bool Simulator::applyRobotCommandVelocity(RobotIdentifier robot, const time::Duration& step_duration,
                                          const Eigen::Vector3d& local_command_velocity) {
    auto pos = this->wm->getTransform(robot.getFrame(), "");
    auto vel = this->wm->getVelocity(robot.getFrame(), "", "");

    if (!pos.has_value() || !vel.has_value()) return false;
    RobotState robot_state = convertToRobotState(pos->transform, vel->velocity);
    robot_state = this->robot_dynamic.applyRobotCommandVelocity(robot_state, local_command_velocity, step_duration);

    auto [new_trans, new_vel] = convertFromRobotState(robot_state);
    transform::TransformWithVelocity trans_with_vel;
    trans_with_vel.header.child_frame = robot.getFrame();
    trans_with_vel.header.parent_frame = "";
    trans_with_vel.header.stamp = current_time + step_duration;
    trans_with_vel.transform = new_trans;
    trans_with_vel.velocity = new_vel;
    this->wm->pushTransform(trans_with_vel);
    return true;
}

Eigen::Vector3d Simulator::getRobotCommandVelocityFromCommand(const robot_interface::RobotCommand& command) {
    if (!command.move_command.has_value()) return {0.0, 0.0, 0.0};
    auto visitor = overload{//
                            [](const robot_interface::RobotVelocityControl& velocity_control) -> Eigen::Vector3d {
                                return velocity_control.desired_velocity;
                            },
                            [](const robot_interface::RobotPositionControl& position_control) -> Eigen::Vector3d {
                                /// @todo convert into local coordinate system
                                return position_control.desired_velocity;
                            }};

    return std::visit(visitor, command.move_command.value());
}

}  // namespace luhsoccer::robot_control
