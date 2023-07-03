#include <memory>

#include "local_planner/simulator.hpp"

#include "local_planner/thread_pool.hpp"
#include "visit.hpp"

namespace luhsoccer::local_planner {
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
unsigned long Simulator::last_simulation_id = 0;
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
std::mutex Simulator::simulation_id_mtx{};

Simulator::Simulator(ThreadPool& thread_pool, const std::shared_ptr<const transform::WorldModel>& base_wm,
                     const std::shared_ptr<AvoidanceManager>& am, RobotDynamic robot_dynamic,
                     double simulation_frequency)
    : wm(std::make_shared<transform::WorldModel>(*base_wm)),  // copy WorldModel
      base_wm(base_wm),
      am(am),
      robot_dynamic(std::move(robot_dynamic)),
      simulation_frequency(simulation_frequency),
      simulation_id(0),
      thread_pool(thread_pool),
      start_time(std::nullopt) {
    const std::lock_guard lock(Simulator::simulation_id_mtx);
    // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
    this->simulation_id = ++Simulator::last_simulation_id;
};

unsigned long Simulator::startSimulation(const std::shared_ptr<Simulator>& simulator,
                                         const std::atomic_bool& should_run, const time::TimePoint& start_time,
                                         const std::optional<time::TimePoint>& start_from) {
    if (!simulator) return 0;
    if (!simulator->start_time.has_value()) {
        if (start_time.asSec() == 0.0) {
            simulator->start_time = time::now();
        } else {
            simulator->start_time = start_time;
        }
    }
    if (!start_from.has_value()) {
        simulator->current_time = simulator->start_time.value();
    } else {
        simulator->current_time = start_from.value();
    }

    if (simulator->tasks.empty()) {
        LOG_WARNING(simulator->logger,
                    "Tasks of simulator with id {:d} is empty, when trying to start simulation. Set tasks first!",
                    simulator->simulation_id);
        return 0;
    }
    if (simulator->state == SimulatorState::RUNNING) return 0;

    simulator->thread_pool.addJob([simulator, &should_run]() { simulator->runSimulation(should_run); });
    return simulator->simulation_id;
}

SimulationResult Simulator::runSyncSimulation(const std::atomic_bool& should_run, const time::TimePoint& start_time,
                                              const std::optional<time::TimePoint>& start_from) {
    if (!this->start_time.has_value()) {
        if (start_time.asSec() == 0.0) {
            this->start_time = time::now();
        } else {
            this->start_time = start_time;
        }
    }

    if (!start_from.has_value()) {
        this->current_time = this->start_time.value();
    } else {
        this->current_time = start_from.value();
    }

    if (this->tasks.empty()) {
        LOG_WARNING(this->logger,
                    "Tasks of this with id {:d} is empty, when trying to start simulation. Set tasks first!",
                    this->simulation_id);
        return {};
    }
    if (this->state == SimulatorState::RUNNING) return {};
    SimulationResult simulation_result;
    this->result_callback = [&simulation_result](unsigned long /*id*/, const SimulationResult& result) {
        simulation_result = result;
    };

    this->runSimulation(should_run);
    return simulation_result;
}

bool isCommandActive(const robot_interface::RobotCommand& cmd) {
    constexpr double SMALL_VALUE = 0.001;
    if (cmd.kick_command.has_value() && cmd.kick_command->kick_velocity > SMALL_VALUE) return true;
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

void Simulator::runSimulation(const std::atomic_bool& should_run) {
    this->agents.clear();

    for (const auto& [robot, task] : this->tasks) {
        auto [agent, success] = this->agents.try_emplace(robot, this->robot_dynamic, robot, simulation_id, this->am,
                                                         this->simulation_frequency, false);
        agent->second.setTask(task.skill, task.task_data);
    }

    bool run = true;
    this->should_run = true;
    // if (!this->start_time.has_value()) this->start_time = time::now();
    // time::TimePoint current_time = this->start_time.value();
    constexpr time::Duration TIMEOUT = 30.0;
    while (run && should_run && this->should_run) {
        auto commands = this->calcCommands();
        for (const auto& cmd : commands) {
            if (!isCommandActive(cmd.second)) {
                if (this->inactive_commands.count(cmd.first) == 0) {
                    this->inactive_commands[cmd.first] = this->current_time;
                }
            } else {
                if (this->inactive_commands.count(cmd.first)) {
                    this->inactive_commands.erase(cmd.first);
                }
            }
        }
        this->applyRobotCommands(commands, this->current_time);

        bool agent_running = false;
        for (auto& [robot, agent] : this->agents) {
            if (agent.getCurrentState() == Agent::AgentState::RUNNING) {
                // agent is not truely running if commands are inactive for one second
                if (this->inactive_commands.count(robot)) {
                    constexpr time::Duration INACTIVE_TIMEOUT = 1.0;
                    if (this->inactive_commands[robot] < this->current_time - INACTIVE_TIMEOUT) {
                        continue;
                    }
                }
                agent_running = true;
                break;
            }
        }
        bool timeout = (this->current_time - this->start_time.value()) > TIMEOUT;
        run = agent_running && this->simulationValid() && !timeout;
        this->current_time += time::Duration(1.0 / this->simulation_frequency);
    }
    if (!should_run || !this->should_run) {
        LOG_INFO(this->logger, "Simulation {:d} received stop command.", this->simulation_id);
        return;
    }
    // for (auto& [robot, agent] : this->agents) {
    //     if (agent.getCurrentState() != Agent::AgentState::FINISHED) {
    //         LOG_WARNING(this->logger, "One or more agents of simulation {} did not exit with state FINISHED!",
    //                     this->simulation_id);
    //         // return;
    //     }
    // }
    // calc result
    // update task_data in tasks from agents
    for (auto& task : this->tasks) {
        auto agent_it = this->agents.find(task.first);
        if (agent_it != this->agents.end()) {
            auto td = agent_it->second.getTaskData();
            if (td.has_value()) task.second.task_data = td.value();
        };
    }
    SimulationResult result{this->wm, this->tasks, this->start_time.value(), current_time};
    if (this->result_callback.has_value()) this->result_callback.value()(this->simulation_id, result);
}

std::unordered_map<RobotIdentifier, robot_interface::RobotCommand> Simulator::calcCommands() {
    std::unordered_map<RobotIdentifier, robot_interface::RobotCommand> commands;
    for (auto& [robot, agent] : this->agents) {
        auto cmd = agent.calcCurrentCommandMessage(this->wm, this->current_time);
        if (cmd.first.has_value()) commands[robot] = cmd.first.value();
    }
    return commands;
}

void Simulator::applyRobotCommands(const std::unordered_map<RobotIdentifier, robot_interface::RobotCommand>& commands,
                                   const time::TimePoint& current_time) {
    // move all robots
    for (const auto& robot : this->wm->getVisibleRobots()) {
        // check if command exists for this robot
        if (robot.isAlly() && commands.find(robot) != commands.end()) {
            auto robot_command_vel = this->getRobotCommandVelocityFromCommand(commands.at(robot));
            this->applyRobotCommandVelocity(this->wm, this->robot_dynamic, robot, current_time,
                                            {1.0 / this->simulation_frequency}, robot_command_vel);
        } else {
            this->moveFrame(this->wm, robot.getFrame(), current_time, {1.0 / this->simulation_frequency});
        }
    }
    // move ball
    this->moveFrame(this->wm, this->wm->getBallFrame(), current_time, {1.0 / this->simulation_frequency});
}

bool Simulator::moveFrame(const std::shared_ptr<transform::WorldModel>& wm, const std::string& frame,
                          const time::TimePoint& current_time, const time::Duration& step_duration,
                          const Eigen::Vector3d& acceleration) {
    auto current_pos = wm->getTransform(frame, "", current_time);
    auto current_vel = wm->getVelocity(frame, "", "", current_time);

    if (!current_pos.has_value() || !current_vel.has_value()) return false;

    Eigen::Vector3d new_vel = current_vel->velocity + acceleration * step_duration.asSec() * 0.0;
    Eigen::Affine2d delta_pos = Eigen::Translation2d(new_vel.head(2) * step_duration.asSec() * 0.0) *
                                Eigen::Rotation2Dd(new_vel[2] * step_duration.asSec() * 0.0);
    Eigen::Affine2d new_pos = current_pos->transform * delta_pos;

    transform::TransformWithVelocity trans_with_vel;
    trans_with_vel.header.child_frame = frame;
    trans_with_vel.header.stamp = current_time + step_duration;
    trans_with_vel.header.parent_frame = "";
    trans_with_vel.transform = new_pos;
    trans_with_vel.velocity = new_vel;
    wm->pushTransform(trans_with_vel);
    return true;
}

bool Simulator::applyRobotCommandVelocity(const std::shared_ptr<transform::WorldModel>& wm,
                                          const RobotDynamic& robot_dynamic, RobotIdentifier robot,
                                          const time::TimePoint& current_time, const time::Duration& step_duration,
                                          const Eigen::Vector3d& local_command_velocity) {
    auto pos = wm->getTransform(robot.getFrame(), "", current_time);
    auto vel = wm->getVelocity(robot.getFrame(), "", "", current_time);

    if (!pos.has_value() || !vel.has_value()) return false;
    RobotState robot_state = convertToRobotState(pos->transform, vel->velocity);
    robot_state = robot_dynamic.applyRobotCommandVelocity(robot_state, local_command_velocity, step_duration);

    auto [new_trans, new_vel] = convertFromRobotState(robot_state);
    transform::TransformWithVelocity trans_with_vel;
    trans_with_vel.header.child_frame = robot.getFrame();
    trans_with_vel.header.parent_frame = "";
    trans_with_vel.header.stamp = current_time + step_duration;
    trans_with_vel.transform = new_trans;
    trans_with_vel.velocity = new_vel;
    wm->pushTransform(trans_with_vel);
    return true;
}

Eigen::Vector3d Simulator::getRobotAccelerationFromCommand(const std::shared_ptr<const transform::WorldModel>& wm,
                                                           const RobotIdentifier& robot,
                                                           const robot_interface::RobotCommand& command,
                                                           const time::Duration& step_duration,
                                                           const time::TimePoint& time) {
    if (!command.move_command.has_value()) return {0.0, 0.0, 0.0};

    auto visitor = overload{
        [&wm, &robot, &step_duration,
         &time](const robot_interface::RobotVelocityControl& velocity_control) -> Eigen::Vector3d {
            const auto& lpc_cs = localPlannerConfig();
            // desired_robot command
            // velocity_control.desired_velocity;
            Eigen::Vector3d desired_local_vel = velocity_control.desired_velocity;
            desired_local_vel.x() = clipVelocity(desired_local_vel.x(), lpc_cs.robot_vel_max_x);
            desired_local_vel.y() = clipVelocity(desired_local_vel.y(), lpc_cs.robot_vel_max_y);
            desired_local_vel.z() = clipVelocity(desired_local_vel.z(), lpc_cs.robot_vel_max_theta);

            // get current velocity and position
            auto current_global_vel = wm->getVelocity(robot.getFrame(), "", "", time);
            auto current_position = wm->getTransform(robot.getFrame(), "", time);
            if (!current_global_vel.has_value() || !current_position.has_value()) return Eigen::Vector3d::Zero();

            auto rotation_matrix = Eigen::Rotation2Dd(current_position->transform.rotation()).toRotationMatrix();

            // current local velocity
            Eigen::Vector3d current_local_vel;
            current_local_vel.head(2) = rotation_matrix.inverse() * current_global_vel->velocity.head(2);
            current_local_vel.z() = current_global_vel->velocity.z();

            // proportional controller
            Eigen::Vector3d velocity_error = desired_local_vel - current_local_vel;
            double k_p_acc = lpc_cs.simulation_acc_k_p / step_duration.asSec();
            Eigen::Vector3d acc = velocity_error * k_p_acc;

            acc.x() = clipAcceleration(acc.x(), current_local_vel.x(), lpc_cs.robot_acc_max_x, lpc_cs.robot_brk_max_x);
            acc.y() = clipAcceleration(acc.y(), current_local_vel.y(), lpc_cs.robot_acc_max_y, lpc_cs.robot_brk_max_y);
            acc.z() = clipAcceleration(acc.z(), current_local_vel.z(), lpc_cs.robot_acc_max_theta,
                                       lpc_cs.robot_brk_max_theta);

            // transform into global coordinates
            acc.head(2) = rotation_matrix * acc.head(2);
            return acc;
        },
        [&wm, &robot, &step_duration,
         &time](const robot_interface::RobotPositionControl& position_control) -> Eigen::Vector3d {
            const auto& lpc_cs = localPlannerConfig();
            // get current velocity and position
            auto current_global_vel = wm->getVelocity(robot.getFrame(), "", "", time);
            auto current_position = wm->getTransform(robot.getFrame(), "", time);
            if (!current_global_vel.has_value() || !current_position.has_value()) return Eigen::Vector3d::Zero();

            auto rotation_matrix = Eigen::Rotation2Dd(current_position->transform.rotation()).toRotationMatrix();

            // desired_robot command
            Eigen::Vector3d desired_local_vel = position_control.desired_velocity;
            desired_local_vel.head(2) = rotation_matrix.inverse() * desired_local_vel.head(2);

            desired_local_vel.x() = clipVelocity(desired_local_vel.x(), lpc_cs.robot_vel_max_x);
            desired_local_vel.y() = clipVelocity(desired_local_vel.y(), lpc_cs.robot_vel_max_y);
            desired_local_vel.z() = clipVelocity(desired_local_vel.z(), lpc_cs.robot_vel_max_theta);

            // current local velocity
            Eigen::Vector3d current_local_vel;
            current_local_vel.head(2) = rotation_matrix.inverse() * current_global_vel->velocity.head(2);
            current_local_vel.z() = current_global_vel->velocity.z();

            // proportional controller
            Eigen::Vector3d velocity_error = desired_local_vel - current_local_vel;
            double k_p_acc = lpc_cs.simulation_acc_k_p / step_duration.asSec();
            Eigen::Vector3d acc = velocity_error * k_p_acc;

            acc.x() = clipAcceleration(acc.x(), current_local_vel.x(), lpc_cs.robot_acc_max_x, lpc_cs.robot_brk_max_x);
            acc.y() = clipAcceleration(acc.y(), current_local_vel.y(), lpc_cs.robot_acc_max_y, lpc_cs.robot_brk_max_y);
            acc.z() = clipAcceleration(acc.z(), current_local_vel.z(), lpc_cs.robot_acc_max_theta,
                                       lpc_cs.robot_brk_max_theta);

            // transform into global coordinates
            acc.head(2) = rotation_matrix * acc.head(2);
            return acc;
        }};

    return std::visit(visitor, command.move_command.value());
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

}  // namespace luhsoccer::local_planner