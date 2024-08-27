#include "robot_controller.hpp"

#include "config/robot_control_config.hpp"
#include "event_system/event_system_helper.hpp"
#include "robot_control/components/component_util.hpp"
#include "robot_control/skills/skill.hpp"

#include "robot_control/components/component_data.hpp"
#include "robot_interface/robot_interface.hpp"

#include "cooperation_module.hpp"

#include "core/visit.hpp"
namespace luhsoccer::robot_control {
RobotController::RobotController(const std::shared_ptr<const transform::WorldModel>& wm, const MarkerAdapter& ma,
                                 const RobotIdentifier& robot, CooperationModule& coop_module)
    : robot_id(robot),
      wm(wm),
      coop_module(coop_module),
      ma(MarkerAdapter(ma)),
      logger(fmt::format("RobotController{}", robot)) {
    this->ma.setManageIDs();
    constexpr double DEFAULT_MARKER_LIFETIME = 0.1;
    this->ma.setOverrideLifetime(DEFAULT_MARKER_LIFETIME);
    this->ma.setOverrideNamespace(fmt::format("RobotController_{}", robot));
    this->logger.setActive(false);
}

bool RobotController::startController(robot_interface::RobotInterface& robot_interface,
                                      event_system::EventSystem& event_system, double controller_freq) {
    if (this->state != RobotControllerState::CREATED) return false;
    this->robot_interface = &robot_interface;
    event_system::helper::registerTimerEventDynamic(
        controller_freq, event_system, [this](const event_system::TimerEvent& /*event*/) { this->update(); });

    this->logger.setActive(true);
    this->state = RobotControllerState::OUT_OF_FIELD;
    return true;
}

bool RobotController::startSimulation() {
    if (this->state != RobotControllerState::CREATED) return false;
    this->state = RobotControllerState::IDLE;
    return true;
}

void RobotController::setActive(bool online) {
    this->state = online ? RobotControllerState::OUT_OF_FIELD : RobotControllerState::OFFLINE;
    if (!online) this->cancelCurrentTask();
}

void RobotController::setSkillInfo(const std::string& skill_info, const Skill* skill,
                                   const std::optional<TaskData>& td) {
    marker::RobotInfo robot_info(robot_id);
    if (this->current_skill_name == skill_info) {
        this->current_skill_repetitions += 1;
        robot_info.addParam("Skill", fmt::format("{} ({}x)", skill_info, this->current_skill_repetitions));
    } else {
        if (this->current_skill_name != "---") {
            this->last_skill_name = this->current_skill_name;
            this->last_skill_repetitions = this->current_skill_repetitions;
        }
        this->current_skill_repetitions = 1;
        this->current_skill_name = skill_info;
        robot_info.addParam("Skill", skill_info);
        if (last_skill_repetitions > 1)
            robot_info.addParam("last Skill",
                                fmt::format("{} ({}x)", this->last_skill_name, this->last_skill_repetitions));
        else
            robot_info.addParam("last Skill", this->last_skill_name);
    }

    // Taskdata
    if (td.has_value() && skill != nullptr) {
        std::stringstream ss;
        size_t i = 0;
        for (auto& position : td->required_positions) {
            std::string point_name = ", ";
            if (i < skill->required_point.size()) {
                point_name = fmt::format("\n  {}:", skill->required_point[i]);
            }
            auto pos_offset = position.getPositionOffset();
            constexpr double RADIAN_TO_DEGREE = 180.0 / L_PI;
            ss << fmt::format("  {} ({}, {:.2f}, {:.2f}, {:.0f}°)", point_name,
                              position.getFrame() == "world" ? "" : position.getFrame(), pos_offset.translation().x(),
                              pos_offset.translation().y(),
                              Eigen::Rotation2Dd(pos_offset.rotation()).angle() * RADIAN_TO_DEGREE);

            i++;
        }
        i = 0;
        for (auto& robot : td->related_robots) {
            if (robot == EMPTY_IDENTIFIER) {
                i++;
                continue;
            }
            std::string robot_name = ", ";
            if (i < skill->related_robot.size()) {
                robot_name = fmt::format("\n  {}:", skill->related_robot[i]);
            }
            ss << fmt::format("{} {}", robot_name, robot);
            i++;
        }
        i = 0;
        for (auto double_val : td->required_doubles) {
            std::string double_name = ", ";
            if (i < skill->required_double.size()) {
                double_name = fmt::format("\n  {}:", skill->required_double[i]);
            }
            ss << fmt::format("{} {}", double_name, double_val);
            i++;
        }
        i = 0;
        for (auto int_val : td->required_ints) {
            std::string int_name = ", ";
            if (i < skill->required_int.size()) {
                int_name = fmt::format("\n  {}:", skill->required_int[i]);
            }
            ss << fmt::format("{} {}", int_name, int_val);
            i++;
        }
        i = 0;
        for (bool bool_val : td->required_bools) {
            std::string bool_name = ", ";
            if (i < skill->required_bool.size()) {
                bool_name = fmt::format("\n  {}:", skill->required_bool[i]);
            }
            ss << fmt::format("{} {}", bool_name, bool_val ? "True" : "False");
            i++;
        }
        i = 0;
        for (auto& string_val : td->required_strings) {
            std::string string_name = ", ";
            if (i < skill->required_string.size()) {
                string_name = fmt::format("\n  {}:", skill->required_string[i]);
            }
            ss << fmt::format("{} {}", string_name, string_val);
            i++;
        }
        robot_info.addParam("TaskData", ss.str());
    }

    this->ma.displayMarker(robot_info);
}

void RobotController::updateSkillText() const {
    const std::shared_lock read_lock(this->task_mtx);
    if (!this->skill) return;
    const double left_shift = static_cast<double>(skill->name.length()) / 2.0 * 0.05;
    constexpr double SKILL_TEXT_OFFSET = 0.15;
    constexpr double SKILL_TEXT_HEIGHT = 0.3;
    constexpr double SKILL_SCALE = 1.5;
    marker::Text skill_text(
        {this->robot_id.getFrame(), -left_shift, SKILL_TEXT_OFFSET}, "", 0,
        fmt::format("{}({:d}/{:d})", this->skill->name, this->current_step + 1, this->skill->steps.size()));
    skill_text.setHeight(SKILL_TEXT_HEIGHT);
    skill_text.setColor(marker::Color::WHITE());
    skill_text.setScale(SKILL_SCALE);
    this->ma.displayMarker(skill_text);
}

bool RobotController::setTask(const Skill* skill, const TaskData& td) {
    if (!skill) return false;
    // check if RobotController is in valid state
    if (this->state != RobotControllerState::IDLE) return false;

    if (!skill->taskDataValid(td)) return false;

    const std::unique_lock write_lock(this->task_mtx);

    this->skill = skill;
    this->task_data = td;
    this->current_step = 0;

    this->state = RobotControllerState::RUNNING;
    this->logger.debug("{} starts skill '{}'.", this->robot_id, this->skill->name);

    this->setSkillInfo(skill->name, this->skill, this->task_data);
    return true;
}

bool RobotController::updateTaskData(const TaskData& td) {
    if (this->state != RobotControllerState::RUNNING) return false;
    const std::unique_lock write_lock(this->task_mtx);
    this->task_data = td;
    return true;
}

bool RobotController::cancelCurrentTask() {
    if (this->state != RobotControllerState::RUNNING) return false;

    this->state = RobotControllerState::IDLE;

    const std::unique_lock write_lock(this->task_mtx);

    this->stopRobot();

    this->skill = nullptr;

    // this->setSkillInfo("---");
    return true;
}

const Skill* RobotController::getSkill() const { return this->skill; }

robot_interface::RobotCommand RobotController::getNullCmd() {
    robot_interface::RobotCommand null_cmd;
    robot_interface::RobotVelocityControl vel_cmd;
    vel_cmd.desired_velocity = {0.0, 0.0, 0.0};
    null_cmd.move_command = vel_cmd;
    robot_interface::KickCommand kick_cmd{0.0};
    kick_cmd.execute_time = robot_interface::KickExecuteTime::NOW;
    null_cmd.kick_command = kick_cmd;
    return null_cmd;
}

void RobotController::sendCommand(const robot_interface::RobotCommand& robot_command) const {
    if (!robot_interface) return;
    auto game_state = this->wm->getGameState();
    if ((game_state.has_value() && game_state.value() == transform::GameState::HALT) ||
        !robotControlConfig().send_commands) {
        auto cmd = this->getNullCmd();
        cmd.dribbler_mode = robot_interface::DribblerMode::OFF;
        this->robot_interface->updateCommand(this->robot_id, cmd);
    } else {
        this->robot_interface->updateCommand(this->robot_id, robot_command);
    }
}

void RobotController::stopRobot() const {
    robot_interface::RobotCommand msg = this->getNullCmd();
    msg.special_mode = robot_interface::StopMovement();
    this->sendCommand(msg);
}

void RobotController::checkIfRobotOnField() {
    auto robot_data = this->wm->getRobotData(this->robot_id);
    bool robot_on_field = robot_data.has_value() && robot_data->on_field;

    if (!robot_on_field && this->state != RobotControllerState::OUT_OF_FIELD &&
        this->state != RobotControllerState::OFFLINE) {
        this->state = RobotControllerState::OUT_OF_FIELD;
        this->cancelCurrentTask();

        this->logger.info("{} not on field. Turning off RobotController!", this->robot_id);
    } else if (robot_on_field && this->state == RobotControllerState::OUT_OF_FIELD) {
        this->state = RobotControllerState::IDLE;
        this->logger.info("{} on field. Turning on RobotController!", this->robot_id);
    }
}

std::pair<std::optional<robot_interface::RobotCommand>, RobotControllerState>
RobotController::calcCurrentCommandMessage(const time::TimePoint& time) {
    const std::unique_lock write_lock(this->task_mtx);
    if (this->state != RobotControllerState::RUNNING || !this->skill || !this->task_data.has_value())
        return {std::nullopt, RobotControllerState::ERROR};

    ComponentData data(this->wm, this->task_data.value(), this->robot_id, this->coop_module, time, this->ma);
    auto [step_state, msg] = this->skill->calcCommandMessage(data, this->current_step);

    switch (step_state) {
        case Skill::SkillState::RUNNING:
            break;
        case Skill::SkillState::NEXT_STEP:
            this->current_step++;
            break;
        case Skill::SkillState::FINISHED:
            this->state = RobotControllerState::FINISHED;
            break;
        case Skill::SkillState::ABORT:
            this->state = RobotControllerState::ABORTED;
            msg = this->getNullCmd();
            break;
        case Skill::SkillState::ERROR:
            this->state = RobotControllerState::ERROR;
            if (this->skill) {
                this->logger.error("Error occurred at RobotController of {} in skill '{}'.", this->robot_id,
                                   this->skill->name);
            } else {
                this->logger.error("Error occurred at RobotController of {}.", this->robot_id);
            }

            msg = this->getNullCmd();
            break;
    }

    this->ma.resetIDs();

    // check for nan
    auto visitor =
        overload{[](const robot_interface::RobotVelocityControl& cmd) { return cmd.desired_velocity.hasNaN(); },
                 [](const robot_interface::RobotPositionControl& cmd) {
                     return cmd.desired_velocity.hasNaN() || !cmd.desired_position.matrix().allFinite();
                 }};

    if (msg.move_command.has_value() && std::visit(visitor, msg.move_command.value())) {
        msg = this->getNullCmd();
        this->state = RobotControllerState::ERROR;
        this->logger.error("NaN in command message of RobotController of {}.", this->robot_id);
    }

    return {msg, this->state};
}

void RobotController::update() {
    this->checkIfRobotOnField();

    switch (this->state) {
        case RobotControllerState::IDLE: {
            this->stopRobot();
            const std::unique_lock write_lock(this->task_mtx);
            if (this->current_skill_name != "---") {
                this->setSkillInfo("---");
            }
        } break;

        case RobotControllerState::RUNNING: {
            auto [msg, _] = this->calcCurrentCommandMessage();
            if (msg) {
                this->sendCommand(*msg);
            } else {
                this->stopRobot();
                this->state = RobotControllerState::ERROR;
                this->logger.error("Error from Skill!");
            }
            this->updateSkillText();
        } break;

        case RobotControllerState::OFFLINE:
        case RobotControllerState::OUT_OF_FIELD: {
            // constexpr double OFFLINE_WAIT_TIME = 0.5;
            // std::this_thread::sleep_for(time::Duration(OFFLINE_WAIT_TIME));
        } break;

        case RobotControllerState::CREATED:
            throw std::runtime_error(fmt::format(
                "RobotController of {} loop function called in created State, which should not be possible!",
                this->robot_id));
            break;

        case RobotControllerState::FINISHED: {
            const std::unique_lock write_lock(this->task_mtx);
            if (this->skill != nullptr) {
                this->logger.debug("{} finished skill '{}'.", this->robot_id, this->skill->name);
            }
            this->setSkillInfo("---");
        }

            this->state = RobotControllerState::IDLE;
            this->stopRobot();

            break;

        case RobotControllerState::ABORTED: {
            const std::unique_lock write_lock(this->task_mtx);
            if (this->skill != nullptr) {
                this->logger.debug("Skill '{}' of {} was aborted.", this->skill->name, this->robot_id);
            }
            this->setSkillInfo("---");
        }

            this->state = RobotControllerState::IDLE;
            this->stopRobot();
            break;

        default:
        case RobotControllerState::ERROR:
            this->stopRobot();
            std::this_thread::sleep_for(time::Duration(1.0));
            this->logger.debug("Setting RobotController for {} to idle state from error after waiting 1s...",
                               this->robot_id);

            {
                const std::unique_lock write_lock(this->task_mtx);
                this->setSkillInfo("---");
            }
            this->state = RobotControllerState::IDLE;
            break;
    }

    marker::RobotInfo info(this->robot_id);
    std::string msg = format_as(this->state);
    info.addParam("RobotControllerState", msg);
    this->ma.displayMarker(info);
}

}  // namespace luhsoccer::robot_control
