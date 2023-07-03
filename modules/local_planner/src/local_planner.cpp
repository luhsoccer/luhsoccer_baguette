
#include "local_planner/local_planner.hpp"
#include "robot_interface/robot_interface.hpp"

namespace luhsoccer::local_planner {

constexpr int SKILL_TEXT_ID = 1;
constexpr double SKILL_TEXT_HEIGHT = 0.3;
constexpr double SKILL_TEXT_OFFSET = 0.1;
constexpr int LAST_RESERVED_ID = 10;

LocalPlanner::LocalPlanner(std::shared_ptr<const transform::WorldModel> wm,
                           const std::shared_ptr<AvoidanceManager>& avoidance_manager,
                           robot_interface::RobotInterface& robot_interface, marker::MarkerService& ms,
                           const RobotIdentifier& robot, std::optional<double> controller_freq)
    : skill(nullptr),
      state(LocalPlannerState::CREATED),
      real_agent(RobotDynamic(), robot, 0, avoidance_manager,
                 controller_freq ? controller_freq.value() : localPlannerConfig().controller_freq, true),
      wm(std::move(wm)),
      robot_interface(robot_interface),
      ms(ms),
      marker_ns(fmt::format("{}_LocalPlanner", robot)),
      robot(robot),
      rate(controller_freq ? controller_freq.value() : localPlannerConfig().controller_freq,
           fmt::format("{}LocalPlannerControlLoop", robot)),
      sw(fmt::format("LocalPlanner{}", robot)),
      logger("local_planner") {}

bool LocalPlanner::setTask(const Skill* skill, const TaskData& td) {
    const std::lock_guard<std::mutex> lock(this->real_agent_mtx);
    if (!skill) return false;
    // check if LocalPlanner is in valid state
    if (this->state != LocalPlannerState::IDLE) return false;

    if (!skill->taskDataValid(td)) return false;

    this->skill = skill;
    this->task_data = td;
    this->real_agent.cancelCurrentTask();
    if (!this->real_agent.setTask(skill, td)) return false;

    this->state = LocalPlannerState::RUNNING;
    LOG_DEBUG(this->logger, "{} starts skill '{}'.", this->robot, this->skill->name);
    const double left_shift = static_cast<double>(skill->name.length()) / 2.0 * 0.05;
    marker::Text skill_text({this->robot.getFrame(), -left_shift, SKILL_TEXT_OFFSET}, marker_ns, SKILL_TEXT_ID,
                            skill->name);
    skill_text.setHeight(SKILL_TEXT_HEIGHT);
    skill_text.setColor(marker::Color::WHITE());
    this->ms.displayMarker(skill_text);
    marker::RobotInfo robot_info(robot);
    robot_info.addParam("Skill", skill->name);
    this->ms.displayMarker(robot_info);
    return true;
}

bool LocalPlanner::cancelCurrentTask() {
    if (this->state == LocalPlannerState::RUNNING) {
        const std::lock_guard<std::mutex> lock(this->real_agent_mtx);
        if (!this->real_agent.cancelCurrentTask()) return false;

        this->state = LocalPlannerState::IDLE;
        robot_interface::RobotCommand msg = this->getNullCmd();
        msg.special_mode = robot_interface::StopMovement();
        this->sendCommand(msg);
        this->ms.deleteMarker(marker_ns, SKILL_TEXT_ID);
        marker::RobotInfo robot_info(robot);
        robot_info.addParam("Skill", std::string("---"));
        this->ms.displayMarker(robot_info);
        return true;
    };
    return false;
}

void LocalPlanner::setState(bool online) {
    this->state = online ? LocalPlannerState::OUT_OF_FIELD : LocalPlannerState::OFFLINE;
    if (!online) {
        const std::lock_guard<std::mutex> lock(this->real_agent_mtx);
        this->real_agent.cancelCurrentTask();
        this->ms.deleteMarker(marker_ns, SKILL_TEXT_ID);
        marker::RobotInfo robot_info(robot);
        robot_info.addParam("Skill", std::string("---"));
        this->ms.displayMarker(robot_info);
    }
}

void LocalPlanner::checkIfRobotOnField() {
    auto robot_data = this->wm->getRobotData(this->robot);
    bool robot_on_field = robot_data.has_value() && robot_data->on_field;

    if (!robot_on_field && this->state != LocalPlannerState::OUT_OF_FIELD &&
        this->state != LocalPlannerState::OFFLINE) {
        this->state = LocalPlannerState::OUT_OF_FIELD;
        const std::lock_guard<std::mutex> lock(this->real_agent_mtx);
        this->real_agent.cancelCurrentTask();
        this->ms.deleteMarker(marker_ns, SKILL_TEXT_ID);
        marker::RobotInfo robot_info(robot);
        robot_info.addParam("Skill", std::string("---"));
        this->ms.displayMarker(robot_info);
        LOG_INFO(this->logger, "{} not on field. Turning off LocalPlanner!", this->robot);
    } else if (robot_on_field && this->state == LocalPlannerState::OUT_OF_FIELD) {
        this->state = LocalPlannerState::IDLE;
        LOG_INFO(this->logger, "{} on field. Turning on LocalPlanner!", this->robot);
    }
}

robot_interface::RobotCommand LocalPlanner::getNullCmd() const {
    robot_interface::RobotCommand null_cmd;
    robot_interface::RobotVelocityControl vel_cmd;
    vel_cmd.desired_velocity = {0.0, 0.0, 0.0};
    null_cmd.move_command = vel_cmd;
    robot_interface::KickCommand kick_cmd;
    kick_cmd.kick_velocity = 0.0;
    kick_cmd.cap_voltage = 0;
    kick_cmd.execute_time = robot_interface::KickExecuteTime::NOW;
    null_cmd.kick_command = kick_cmd;
    return null_cmd;
}

void LocalPlanner::sendCommand(const robot_interface::RobotCommand& msg) {
    auto game_state = this->wm->getGameState();
    if (game_state.has_value() && game_state.value() == transform::GameState::HALT) {
        auto cmd = this->getNullCmd();
        cmd.dribbler_mode = robot_interface::DribblerMode::OFF;
        this->robot_interface.updateCommand(this->robot, cmd);
    } else {
        this->robot_interface.updateCommand(this->robot, msg);
    }
}

bool LocalPlanner::startLoop(const std::atomic_bool& should_run) {
    if (this->state == LocalPlannerState::CREATED) {
        this->state = LocalPlannerState::OUT_OF_FIELD;
        this->join();
        this->thread = std::thread(&LocalPlanner::loop, this, std::ref(should_run));
        LOG_DEBUG(this->logger, "Started local planner for {}", this->robot);
        return true;
    }
    return false;
}

void LocalPlanner::join() {
    if (this->thread.joinable()) {
        this->thread.join();
    }
}

void LocalPlanner::loop(const std::atomic_bool& should_run) {
    while (should_run) {
        // bool print_results = false;
        // this->sw.startSection("loop");
        this->checkIfRobotOnField();

        switch (this->state) {
            case LocalPlannerState::IDLE: {
                this->sendNullCommand();
            } break;

            case LocalPlannerState::RUNNING: {
                // print_results = true;
                // sw.startSection("running state");
                this->real_agent_mtx.lock();
                // sw.startSection("calcCurrentCommandMessage");
                auto [msg, agent_state] = this->real_agent.calcCurrentCommandMessage(this->wm);
                // sw.endSection("calcCurrentCommandMessage");
                // sw.startSection("getMarkers");
                std::vector<Marker> markers = this->real_agent.getMarkers(this->wm);
                // sw.endSection("getMarkers");
                this->real_agent_mtx.unlock();

                switch (agent_state) {
                    case Agent::AgentState::IDLE:
                        this->state = LocalPlannerState::IDLE;
                        break;
                    case Agent::AgentState::RUNNING:
                        // send data;
                        break;

                    case Agent::AgentState::FINISHED: {
                        this->state = LocalPlannerState::IDLE;
                        LOG_DEBUG(this->logger, "{} finished skill '{}'.", this->robot, this->skill->name);
                        this->ms.deleteMarker(marker_ns, SKILL_TEXT_ID);
                        marker::RobotInfo robot_info(robot);
                        robot_info.addParam("Skill", std::string("---"));
                        this->ms.displayMarker(robot_info);
                        break;
                    }
                    case Agent::AgentState::ABORTED:
                        this->state = LocalPlannerState::IDLE;
                        msg = this->getNullCmd();
                        msg->special_mode = robot_interface::StopMovement();
                        LOG_DEBUG(this->logger, "Skill '{}' of {} was aborted.", this->skill->name, this->robot);
                        break;
                    case Agent::AgentState::ERROR:
                        this->state = LocalPlannerState::ERROR;
                        msg = this->getNullCmd();
                        if (this->skill) {
                            LOG_ERROR(this->logger, "Error occurred at LocalPlanner of {} in skill '{}'.", this->robot,
                                      this->skill->name);
                        } else {
                            LOG_ERROR(this->logger, "Error occurred at LocalPlanner of {}.", this->robot);
                        }
                        break;
                }
                if (msg) {
                    this->sendCommand(*msg);
                } else {
                    this->sendNullCommand();
                    this->state = LocalPlannerState::ERROR;
                }
                // publish marker
                // sw.startSection("displayMarker");
                int id = LAST_RESERVED_ID;
                for (auto& m : markers) {
                    auto visitor = overload{[this, &id](marker::Marker& marker) {
                                                marker.setNs(this->marker_ns);
                                                marker.setId(++id);
                                                constexpr double DEFAULT_MARKER_LIFETIME = 0.1;
                                                marker.setLifetime(DEFAULT_MARKER_LIFETIME);
                                                this->ms.displayMarker(marker);
                                            },
                                            [this](marker::LinePlot& marker) { this->ms.displayMarker(marker); }};
                    std::visit(visitor, m);
                }
                // LOG_INFO(this->logger, "Set {:d} markers for robot {}", markers.size(), this->robot);
                // sw.endSection("displayMarker");

                // sw.endSection("running state");
            } break;

            case LocalPlannerState::OFFLINE:
            case LocalPlannerState::OUT_OF_FIELD: {
                constexpr double OFFLINE_WAIT_TIME = 0.5;
                std::this_thread::sleep_for(time::Duration(OFFLINE_WAIT_TIME));
            }
                // sw.endSection("loop");
                continue;
                break;

            case LocalPlannerState::ERROR:
                break;
            case LocalPlannerState::CREATED:
                throw std::runtime_error(fmt::format(
                    "LocalPlanner's of {} loop function called in created State, which should not be possible!",
                    this->robot));
                break;
        }
        // this->sw.endSection("loop");
        // if (print_results) this->sw.printSectionTimes();
        this->rate.sleep();
    }
    LOG_DEBUG(this->logger, "Stopped local planner for {}", this->robot);
}

}  // namespace luhsoccer::local_planner