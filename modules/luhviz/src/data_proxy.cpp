#include "include/data_proxy.hpp"
#include <numeric>
#include <transform_helper/world_model_helper.hpp>

#include "config/game_config.hpp"
#include "scenario/scenario_book.hpp"

namespace luhsoccer::luhviz {

DataProxy::DataProxy(software_manager::SoftwareManager& sm, ssl_interface::SSLInterface& ssl,
                     simulation_interface::SimulationInterface& sim, robot_control::RobotControlModule& robot_control,
                     skills::SkillLibrary& skill_lib, robot_interface::RobotInterface& robot_interface,
                     game_data_provider::GameDataProvider& gdp, scenario::ScenarioExecutor& scenario_executor)
    : software_manager(sm),
      ssl(ssl),
      robot_interface(robot_interface),
      sim(sim),
      robot_control(robot_control),
      skill_lib(skill_lib),
      selected_vision_source(ssl_interface::VisionDataSource::DISABLED),
      gdp(gdp),
      scenario_executor(scenario_executor) {
    this->ssl.setVisionDataSource(this->selected_vision_source);
    loadAllConfigs();

    this->movement_velocity = getConfigDouble("luhviz_internal", "selected_controler_velocity");
    this->zero_kick_time = getConfigDouble("luhviz", "zero_kick_delay");
}

void DataProxy::replayGamelog(std::string file) {
    logger.info("Loading gamelog: {}", file[0]);
    ssl_interface::LogFile log_file(std::move(file));

    ssl.setVisionDataSource(ssl_interface::VisionDataSource::GAME_LOG);
    ssl.setGameControllerDataSource(ssl_interface::GameControllerDataSource::GAME_LOG);
    ssl.playGameLog(std::move(log_file));
}

void DataProxy::setVisionSource(const std::string& src) {
    if (fmt::to_string(this->selected_vision_source) == src)
        return;
    else {
        if (src == "Simulation") {
            this->selected_vision_source = ssl_interface::VisionDataSource::SIMULATION;
        } else if (src == "Network") {
            this->selected_vision_source = ssl_interface::VisionDataSource::NETWORK;
        } else if (src == "Game-Log") {
            this->selected_vision_source = ssl_interface::VisionDataSource::GAME_LOG;
        } else {
            this->selected_vision_source = ssl_interface::VisionDataSource::DISABLED;
        }
    }

    this->ssl.setVisionDataSource(this->selected_vision_source);
}

std::string DataProxy::getVisionSource() { return fmt::to_string(this->selected_vision_source); }

void DataProxy::setRobotConnection(const std::string& src) {
    if (fmt::to_string(this->selected_robot_connection) == src)
        return;
    else {
        if (src == "Disabled") {
            this->selected_robot_connection = robot_interface::RobotConnection::DISABLED;
        } else if (src == "Network") {
            this->selected_robot_connection = robot_interface::RobotConnection::NETWORK;
        } else if (src == "Serial") {
            this->selected_robot_connection = robot_interface::RobotConnection::SERIAL;
        } else if (src == "Simulation") {
            this->selected_robot_connection = robot_interface::RobotConnection::SIMULATION;
        }
    }

    this->robot_interface.setConnectionType(this->selected_robot_connection);
}

std::string DataProxy::getRobotConnection() { return fmt::to_string(this->selected_robot_connection); }

void DataProxy::setSimulationConnection(const std::string& src) {
    if (fmt::to_string(this->selected_simulation_connector) == src)
        return;
    else {
        if (src == "None") {
            this->selected_simulation_connector = simulation_interface::SimulationConnectorType::NONE;
        } else if (src == "Test-Simulation") {
            this->selected_simulation_connector = simulation_interface::SimulationConnectorType::TEST_SIMULATION;
        } else if (src == "ErForce-Simulation") {
            this->selected_simulation_connector = simulation_interface::SimulationConnectorType::ERFORCE_SIMULATION;
        } else if (src == "ErSim") {
            this->selected_simulation_connector = simulation_interface::SimulationConnectorType::ER_SIM;
        }
    }

    this->sim.switchConnector(this->selected_simulation_connector);
}

std::string DataProxy::getSimulationConnection() { return fmt::to_string(this->selected_simulation_connector); }

void DataProxy::setGameControllerDataSource(const std::string& src) {
    if (fmt::to_string(this->selected_gamecontroller_data_source) == src)
        return;
    else {
        if (src == "Disabled") {
            this->selected_gamecontroller_data_source = ssl_interface::GameControllerDataSource::DISABLED;
        } else if (src == "Network") {
            this->selected_gamecontroller_data_source = ssl_interface::GameControllerDataSource::NETWORK;
        } else if (src == "Internal") {
            this->selected_gamecontroller_data_source = ssl_interface::GameControllerDataSource::INTERNAL;
        } else {
            this->selected_gamecontroller_data_source = ssl_interface::GameControllerDataSource::GAME_LOG;
        }
    }

    this->ssl.setGameControllerDataSource(this->selected_gamecontroller_data_source);
}

std::string DataProxy::getGameControllerDataSource() {
    return fmt::to_string(this->selected_gamecontroller_data_source);
}

void DataProxy::setVisionPublishMode(const std::string& src) {
    if (fmt::to_string(this->selected_vision_publish_mode) == src)
        return;
    else {
        if (src == "Disabled") {
            this->selected_vision_publish_mode = ssl_interface::VisionPublishMode::DISABLED;
        } else {
            this->selected_vision_publish_mode = ssl_interface::VisionPublishMode::NETWORK;
        }
    }

    this->ssl.setVisionPublishMode(this->selected_vision_publish_mode);
}

std::string DataProxy::getVisionPublishMode() { return fmt::to_string(this->selected_vision_publish_mode); }

void DataProxy::teleportBall(const Eigen::Affine2d& target, const Eigen::Vector3d& velocity) {
    Eigen::Affine2d target_copy = target;
    if (config_provider::ConfigProvider::getConfigStore().game_config.is_flipped) {
        target_copy.translation().x() *= -1.0;
        target_copy.translation().y() *= -1.0;
    }
    this->sim.teleportBall(target_copy, velocity);
}

void DataProxy::teleportRobot(size_t robot_id, TeamColor team_color, const Eigen::Affine2d& target, bool present) {
    if (!config_provider::ConfigProvider::getConfigStore().game_config.is_blue) {
        team_color = getOppositeTeamColor(team_color);
    }

    Eigen::Affine2d target_copy = target;
    if (config_provider::ConfigProvider::getConfigStore().game_config.is_flipped) {
        target_copy.translation().x() *= -1.0;
        target_copy.translation().y() *= -1.0;
        target_copy.rotate(Eigen::Rotation2Dd(L_PI));
    }

    this->sim.teleportRobot(static_cast<unsigned int>(robot_id), team_color, target_copy, {0., 0., 0.}, present);
}

void DataProxy::loadAvailableSkills() {
    using namespace std::string_literals;

    // get all available skills and save them
    std::vector<std::pair<std::string, skills::SkillNames>> available_skillnames = skill_lib.getSkillList();
    for (const auto& pair : available_skillnames) {
        auto skill = this->skill_lib.getSkill(pair.second);
        this->available_skills.emplace_back(skill);
        this->available_skills_names += skill.name + "\0"s;
    }
    this->available_skills_names += "\0"s;
    this->selected_skill = &this->available_skills[0];
    this->skills_loaded = true;
}

std::optional<Eigen::Affine2d> DataProxy::getCurrentBallPos() {
    auto tf = this->gdp.getWorldModel()->getTransform(this->gdp.getWorldModel()->getBallFrame());
    if (tf.has_value()) {
        return tf->transform;
    }
    return std::nullopt;
}

bool DataProxy::sendSkillsToRobots() {
    bool success = true;
    if (this->second_skill_enabled) {
        // cancel current skill of that robot
        if (task_data2.has_value()) {
            this->cancelSkill(this->task_data2->robot);
        }

        if (checkTaskData2()) {
            // send the new skill
            success &= this->robot_control.setTask(this->selected_skill2.value(), this->task_data2.value());
            if (success) {
                this->td_positions2.clear();
                this->td_related_robots2.clear();
            }
        }
    }

    // cancel current skill of that robot
    if (task_data.has_value()) {
        this->cancelSkill(this->task_data->robot);
    }

    if (checkTaskData()) {
        // send the new skill
        success &= this->robot_control.setTask(this->selected_skill.value(), this->task_data.value());
        if (success) {
            this->td_positions.clear();
            this->td_related_robots.clear();
        }
        return success;
    }
    return false;
}

bool DataProxy::cancelSkills() {
    bool success = false;
    if (this->selected_robot.has_value()) {
        success = this->robot_control.cancelTask(this->selected_robot.value());
    }
    if (this->second_skill_enabled && this->selected_robot2.has_value()) {
        success &= this->robot_control.cancelTask(this->selected_robot2.value());
    }
    return success;
}

bool DataProxy::cancelSkill(RobotIdentifier& robot) { return this->robot_control.cancelTask(robot); }

std::optional<robot_control::Skill*> DataProxy::setSelectedSkill(std::optional<size_t> skill_index) {
    if (skill_index.has_value()) {
        // ignore if skill has not changed
        robot_control::Skill& skill_sel = this->available_skills[skill_index.value()];
        if (this->selected_skill.has_value() && skill_sel.name.compare(this->selected_skill.value()->name) == 0) {
            return this->selected_skill.value();
        }

        // else apply selected skill
        for (auto& skill : this->available_skills) {
            if (skill.name.compare(skill_sel.name) == 0) {
                this->selected_skill = &skill;
                return &skill;
            }
        }
    }

    return std::nullopt;
}

std::optional<robot_control::Skill*> DataProxy::setSelectedSkill2(std::optional<size_t> skill_index) {
    if (skill_index.has_value()) {
        // ignore if skill has not changed
        robot_control::Skill& skill_sel = this->available_skills[skill_index.value()];
        if (this->selected_skill2.has_value() && skill_sel.name.compare(this->selected_skill2.value()->name) == 0) {
            return this->selected_skill2.value();
        }

        // else apply selected skill
        for (auto& skill : this->available_skills) {
            if (skill.name.compare(skill_sel.name) == 0) {
                this->selected_skill2 = &skill;
                return &skill;
            }
        }
    }

    return std::nullopt;
}

size_t DataProxy::getRemainingPointsToChoose() {
    if (this->task_data.has_value() && this->selected_skill.has_value()) {
        int required_points = 0;
        if (this->selected_skill.value()->required_point_num.has_value()) {
            required_points = static_cast<int>(this->selected_skill.value()->required_point_num.value());
        } else {
            return 1;
        }
        int added_points = static_cast<int>(this->td_positions.size());
        return required_points - added_points;
    }
    logger.warning("Skill is not valid, select a Skill please");
    return 0;
}

size_t DataProxy::getRemainingPointsToChoose2() {
    if (this->task_data2.has_value() && this->selected_skill2.has_value()) {
        int required_points = 0;
        if (this->selected_skill2.value()->required_point_num.has_value()) {
            required_points = static_cast<int>(this->selected_skill2.value()->required_point_num.value());
        }
        int added_points = static_cast<int>(this->td_positions2.size());
        return required_points - added_points;
    }
    return 0;
}

int DataProxy::getCountRelRobotsSelected() {
    if (this->selected_skill.has_value()) {
        int already_set = 0;
        for (const auto& r : this->td_related_robots) {
            if (r.has_value()) ++already_set;
        }
        return already_set;
    }

    logger.warning("Skill is not valid, select a Skill please");
    return 0;
}

int DataProxy::getCountRelRobotsSelected2() {
    if (this->selected_skill2.has_value()) {
        int already_set = 0;
        for (const auto& r : this->td_related_robots2) {
            if (r.has_value()) ++already_set;
        }
        return already_set;
    }

    logger.warning("Skill is not valid, select a Skill please");
    return 0;
}

void DataProxy::updateNextTDRelatedRobot(const RobotIdentifier& related_robot) {
    bool already_selected = false;
    size_t i = 0;
    size_t found_index = 0;
    // check if robot is already selected
    for (const auto& r : this->td_related_robots) {
        if (r.has_value()) {
            if (r.value() == related_robot && r.value().getTeam() == related_robot.getTeam()) {
                already_selected = true;
                found_index = i;
                break;
            }
        }
        ++i;
    }

    if (already_selected && found_index < this->td_related_robots.size()) {
        // deselect if already selected
        this->td_related_robots[found_index] = std::nullopt;
    } else {
        // set the related robot to be there first which is not set yet
        for (auto& r : this->td_related_robots) {
            if (!r.has_value()) {
                r = related_robot;
                break;
            }
        }
    }
}

void DataProxy::clearTaskData() {
    this->td_positions.clear();
    this->td_related_robots.clear();
    this->task_data.reset();
}

bool DataProxy::checkTaskData() {
    if (!this->selected_robot.has_value()) {
        logger.warning("Select a robot to run a skill");
        this->manipulation_mode = ManipulationMode::SELECT;
        return false;
    }

    if (!this->task_data.has_value()) {
        logger.warning("TaskData is not valid, check if all required data are set correctly");
        this->manipulation_mode = ManipulationMode::SELECT;
        return false;
    }

    if (!this->selected_skill.has_value()) {
        logger.warning("Skill is not valid, select a Skill please");
        this->manipulation_mode = ManipulationMode::SELECT;
        return false;
    }

    // add points to taskdata
    std::vector<transform::Position> required_positions{};
    for (const auto& pos : this->td_positions) {
        auto p = transform::Position{"world", pos.x, pos.y, pos.z};  // TODO: change this to non hardcoded frame name
        required_positions.emplace_back(p);
    }
    this->task_data->required_positions = required_positions;

    // add related robots to taskdata
    std::vector<RobotIdentifier> related_robots{};
    for (const auto& rel_robot : this->td_related_robots) {
        if (rel_robot.has_value()) {
            related_robots.emplace_back(rel_robot.value());
        }
    }
    this->task_data->related_robots = related_robots;

    // save task_data if valid
    bool valid = this->selected_skill.value()->taskDataValid(this->task_data.value());
    return valid;
}

bool DataProxy::checkTaskData2() {
    if (!this->selected_robot2.has_value()) {
        logger.warning("Select a robot to run a skill 2");
        this->manipulation_mode = ManipulationMode::SELECT;
        return false;
    }

    if (!this->task_data2.has_value()) {
        logger.warning("TaskData 2 is not valid, check if all required data are set correctly");
        this->manipulation_mode = ManipulationMode::SELECT;
        return false;
    }

    if (!this->selected_skill2.has_value()) {
        logger.warning("Skill 2 is not valid, select a Skill please");
        this->manipulation_mode = ManipulationMode::SELECT;
        return false;
    }

    // add points to taskdata
    std::vector<transform::Position> required_positions{};
    for (const auto& pos : this->td_positions2) {
        auto p = transform::Position{"world", pos.x, pos.y, pos.z};  // TODO change this to non hardcoded frame name
        required_positions.emplace_back(p);
    }
    this->task_data2->required_positions = required_positions;

    // add related robots to taskdata
    std::vector<RobotIdentifier> related_robots{};
    for (const auto& rel_robot : this->td_related_robots2) {
        if (rel_robot.has_value()) {
            related_robots.emplace_back(rel_robot.value());
        }
    }
    this->task_data2->related_robots = related_robots;

    // save task_data if valid
    bool valid = this->selected_skill2.value()->taskDataValid(this->task_data2.value());
    return valid;
}

// gamepad stuff
GamepadStatus& DataProxy::getGamepadStatus() { return this->gamepad_status; }

using namespace std::chrono;
void DataProxy::update() {
    // if (this->dribbler_high && this->controlled_robot.has_value()) {
    //     RobotIdentifier& id = this->controlled_robot.value();
    //     this->robot_interface.updateDribbler(id, robot_interface::DribblerMode::HIGH);
    // }

    if (duration_cast<seconds>(steady_clock::now() - this->last_game_info_update_time) >= seconds(1)) {
        updateGameInfoData();
        this->last_game_info_update_time = steady_clock::now();
    }
}

void DataProxy::updateGameInfoData() {
    const auto game_state = this->gdp.getWorldModel()->getGameState(0);
    if (game_state.has_value()) {
        this->game_data.game_state = game_state.value();
    }
    this->game_data.ally_info = this->gdp.getAllyTeamInfo();
    this->game_data.enemy_info = this->gdp.getEnemyTeamInfo();
}

void DataProxy::gamepadButtonInput(GamepadControls input, int button_state, size_t id, double current_time) {
    if (this->controller_data.find(id) == this->controller_data.end()) return;
    PerControllerData& data = this->controller_data.at(id);

    if (input == GamepadControls::BUTTON_DRIBBLER && button_state == 0) {
        data.gamepad_dribbler_buttonstate = 0;
        return;
    }

    if (((input == GamepadControls::BUTTON_GET_BALL || input == GamepadControls::BUTTON_GO_TO_GOAL) &&
         button_state == 0) ||
        time::Duration(time::now() - gamepad_skill_activated) < gamepad_skill_active_duration) {
        return;
    }

    if (input == GamepadControls::BUTTON_DRIBBLER && button_state == 0) {
        data.gamepad_dribbler_buttonstate = 0;
        return;
    }

    if (input == GamepadControls::BUTTON_VOLTAGE_UP && button_state == 0) {
        data.gamepad_kick_velocity_up_buttonstate = 0;
        return;
    }

    if (input == GamepadControls::BUTTON_VOLTAGE_DOWN && button_state == 0) {
        data.gamepad_velocity_down_buttonstate = 0;
        return;
    }

    if (input == GamepadControls::BUTTON_TOGGLE_KICKER_CHIPPER && button_state == 0) {
        data.gamepad_chipper_buttonstate = 0;
        return;
    }

    if (!data.robot_id.has_value()) return;
    RobotIdentifier robot = data.robot_id.value();

    // update GUI
    this->setGamepadStatus(GamepadStatus::INPUT_RECEIVED);

    PerRobotControlData& control_states = this->getPerRobotControlerData(robot);

    if (input == GamepadControls::BUTTON_VOLTAGE_UP) {
        if (data.gamepad_kick_velocity_up_buttonstate == button_state || button_state == 0) return;
        this->kick_velocity += this->KICK_VELOCITY_STEPS;
        if (this->kick_velocity >= this->MAX_KICK_VELOCITY) {
            this->kick_velocity = this->MAX_KICK_VELOCITY;
        }
        logger.info("Kick velocity raised to: {}m/s", this->kick_velocity);
        data.gamepad_kick_velocity_up_buttonstate = button_state;
    }

    if (input == GamepadControls::BUTTON_VOLTAGE_DOWN) {
        if (data.gamepad_velocity_down_buttonstate == button_state || button_state == 0) return;
        this->kick_velocity -= this->KICK_VELOCITY_STEPS;
        if (this->kick_velocity < 0) {
            this->kick_velocity = 0;
        }
        logger.info("Kick velocity lowered to: {}m/s", this->kick_velocity);
        data.gamepad_velocity_down_buttonstate = button_state;
    }

    if (input == GamepadControls::BUTTON_KICKER) {
        float kick_vel = this->chipper_on ? this->MAX_KICK_VELOCITY : this->kick_velocity;
        robot_interface::KickCommand kick_command = {
            kick_vel, this->chipper_on ? robot_interface::KickType::CHIP : robot_interface::KickType::KICK};
        this->robot_interface.updateKickCommand(robot, kick_command);
        control_states.last_kick_time = current_time;
        logger.info("executed kick command vel: {}, type: {}", kick_command.velocity,
                    kick_command.type == robot_interface::KickType::KICK ? "kick" : "chip");

    } else if (input == GamepadControls::BUTTON_DRIBBLER) {
        if (data.gamepad_dribbler_buttonstate == button_state) return;

        // PerRobotControlData dribblerData{};

        if (control_states.last_dribbler == robot_interface::DribblerMode::OFF) {
            control_states.last_dribbler = robot_interface::DribblerMode::HIGH;
            this->dribbler_high = true;

            logger.info("Dribbler HIGH");
            // dribblerData.last_dribbler = robot_interface::DribblerMode::HIGH;
        } else {
            control_states.last_dribbler = robot_interface::DribblerMode::OFF;
            this->dribbler_high = false;
            logger.info("Dribbler OFF");
        }

        // this->control_states_per_robot.insert_or_assign(robot, dribblerData);

        data.gamepad_dribbler_buttonstate = button_state;
        this->robot_interface.updateDribbler(robot, control_states.last_dribbler);
    } else if (input == GamepadControls::BUTTON_GET_BALL) {
        // send getBall skill
        setLocalPlannerDisabledForRobot(robot, false);
        logger.info("GET BALL");

        // If GetBall already set and A pressed again, cancel
        auto current_skill = this->robot_control.getSkill(robot);
        if (current_skill != nullptr && current_skill->name == "GetBall") {
            this->robot_control.cancelTask(robot);
            gamepad_skill_activated = time::now();
            return;
        }
        this->robot_control.cancelTask(robot);
        robot_control::TaskData task_data{robot};
        this->robot_control.setTask(&this->skill_lib.getSkill(skills::GameSkillNames::GET_BALL), task_data);
        gamepad_skill_activated = time::now();

    } else if (input == GamepadControls::BUTTON_GO_TO_GOAL) {
        // send goToPoint skill
        setLocalPlannerDisabledForRobot(robot, false);
        logger.info("Go TO GOAL");
        this->robot_control.cancelTask(robot);

        auto left_corner = this->gdp.getWorldModel()->getTransform("field_defense_area_corner_ally_left");
        auto right_corner = this->gdp.getWorldModel()->getTransform("field_defense_area_corner_ally_right");
        if (left_corner.has_value() && right_corner.has_value()) {
            // calculate position in the center of the goal area
            Eigen::Vector2d mid_translation =
                (left_corner.value().transform.translation() + right_corner.value().transform.translation()) / 2;
            transform::Position middle_position = {"", mid_translation.x(), mid_translation.y()};

            robot_control::TaskData task_data{robot, {}, {middle_position}};
            this->robot_control.setTask(&this->skill_lib.getSkill(skills::GameSkillNames::GO_TO_POINT), task_data);
            gamepad_skill_activated = time::now();
        }
    } else if (input == GamepadControls::BUTTON_TOGGLE_KICKER_CHIPPER) {
        if (data.gamepad_chipper_buttonstate == button_state || button_state == 0) return;

        this->chipper_on = !this->chipper_on;

        data.gamepad_chipper_buttonstate = button_state;
        logger.info("Chipper ON: {}", this->chipper_on);
    }
}

void DataProxy::publishRobotData(double current_time) {
    for (auto& robot_state : this->control_states_per_robot) {
        this->robot_interface.updateDribbler(robot_state.first, robot_state.second.last_dribbler);
        if (robot_state.second.last_kick_time.has_value()) {
            double last_time = robot_state.second.last_kick_time.value();
            this->zero_kick_time = getConfigDouble("luhviz", "zero_kick_delay");
            logger.debug("{}", zero_kick_time);
            if ((current_time - last_time) >= this->zero_kick_time) {
                robot_interface::KickCommand kick_command{0.0};
                this->robot_interface.updateKickCommand(robot_state.first, kick_command);
                robot_state.second.last_kick_time.reset();
            }
        }
    }
}

float& DataProxy::getMovementVelocity() { return this->movement_velocity; }

float& DataProxy::getRotationVelocity() { return this->rotation_velocity; }

int& DataProxy::getGlobalMovementDir() { return this->global_movement_direction; }

void DataProxy::gamepadAxesInput(float x, float y, float rx, float ry, float rot, size_t id, bool global_movement,
                                 bool point_based_movement) {
    if (this->controller_data.find(id) == this->controller_data.end()) return;
    PerControllerData& data = this->controller_data.at(id);

    if (!data.robot_id.has_value()) return;
    RobotIdentifier robot = data.robot_id.value();

    // Mitigate Controller jiggle
    if (x <= 0.15f && x >= -0.15f) {
        x = 0;
    }
    if (y <= 0.15f && y >= -0.15f) {
        y = 0;
    }

    if (rx <= 0.15f && rx >= -0.15f) {
        rx = 0;
    }
    if (ry <= 0.15f && ry >= -0.15f) {
        ry = 0;
    }

    // Keep old look direction if no direction is 'pressed'
    if (rx <= 0.5f && rx >= -0.5f && ry <= 0.5f && ry >= -0.5f) {
        rx = data.last_rx;
        ry = data.last_ry;
    }

    auto look_dir = Eigen::Vector2d(rx, ry);

    if (!point_based_movement && rot == data.last_rot && x == data.last_x && y == data.last_y && x == 0 && y == 0 &&
        rot == 0 && rx == x && ry == y && rx == data.last_rx && ry == data.last_ry) {
        return;
    }

    data.last_x = x;
    data.last_y = y;
    data.last_rx = look_dir.x();
    data.last_ry = look_dir.y();
    data.last_rot = rot;

    static bool point_based_movement_was_active = false;

    auto wm_push_pos_and_heading = [](game_data_provider::GameDataProvider& gdp, const RobotIdentifier& robot,
                                      const Eigen::Vector2d& pos, const Eigen::Vector2d& heading) {
        Eigen::Affine2d tp = Eigen::Translation2d(pos) * Eigen::Rotation2Dd(0);
        transform::TransformWithVelocity transform_t = {
            .header = {robot.getFrame().append("_controller_position"), "", time::now()},
            .transform = tp,
            .velocity = Eigen::Vector3d{0, 0, 0}};
        gdp.pushControllerTransform(transform_t);
        Eigen::Affine2d hp = Eigen::Translation2d(heading) * Eigen::Rotation2Dd(0);
        transform::TransformWithVelocity transform_h = {
            .header = {robot.getFrame().append("_controller_heading"), "", time::now()},
            .transform = hp,
            .velocity = Eigen::Vector3d{0, 0, 0}};
        gdp.pushControllerTransform(transform_h);
    };

    if (!point_based_movement && point_based_movement_was_active) {
        point_based_movement_was_active = false;
        for (const auto& robot : this->gdp.getWorldModel()->getPossibleRobots()) {
            wm_push_pos_and_heading(this->gdp, robot, {-10, -10}, {-10, -10});
        }
    }

    if (point_based_movement) {
        point_based_movement_was_active = true;
        setLocalPlannerDisabledForRobot(robot, false);

        auto pos_and_rot = transform::RobotHandle(robot, gdp.getWorldModel()).getPosAndRotVec();
        if (pos_and_rot.has_value()) {
            // rotate global movement direction
            auto theta = 0.0;
            switch (this->global_movement_direction) {
                case MOVEMENT_DIR_NORTH:
                    theta = -L_PI / 2;
                    break;
                case MOVEMENT_DIR_SOUTH:
                    theta = L_PI / 2;
                    break;
                case MOVEMENT_DIR_WEST:
                    theta = L_PI;
                    break;
                default:
                    break;
            }

            Eigen::Vector2d d(x, y);
            Eigen::Matrix2d r;
            r << cos(theta), -sin(theta), sin(theta), cos(theta);
            auto rotated = r * d;
            look_dir = r * look_dir;
            x = rotated.x() / 2.0;
            y = rotated.y() / 2.0;
        }

        // If in GetBall and controller was pressed, cancel GetBall
        auto current_skill = this->robot_control.getSkill(robot);
        if (current_skill != nullptr && current_skill->name != "ControllerGoToPoint") {
            if (x != 0 || y != 0) this->robot_control.cancelTask(robot);
        }

        // Set skill if not already set
        if (this->robot_control.getState(robot) != robot_control::RobotControllerState::RUNNING &&
            this->robot_control.getState(robot) != robot_control::RobotControllerState::OUT_OF_FIELD) {
            robot_control::TaskData task_data{robot};
            task_data.required_positions.emplace_back(
                transform::Position(robot.getFrame().append("_controller_position")));
            task_data.required_positions.emplace_back(
                transform::Position(robot.getFrame().append("_controller_heading")));
            this->robot_control.setTask(&this->skill_lib.getSkill(skills::GameSkillNames::CONTROLLER_GO_TO_POINT),
                                        task_data);
        }

        // Update target- and heading point
        Eigen::Vector2d target_point = pos_and_rot->head<2>() + Eigen::Vector2d(x, y);
        Eigen::Vector2d heading_point = pos_and_rot->head<2>() + Eigen::Vector2d(look_dir.x(), look_dir.y());
        wm_push_pos_and_heading(this->gdp, robot, target_point, heading_point);

        return;
    } else if (global_movement) {
        auto pos_and_rot = transform::RobotHandle(robot, gdp.getWorldModel()).getPosAndRotVec();
        if (pos_and_rot.has_value()) {
            double theta = -pos_and_rot->z();

            // rotate global movement direction
            switch (this->global_movement_direction) {
                case MOVEMENT_DIR_NORTH:
                    theta -= L_PI / 2;
                    break;
                case MOVEMENT_DIR_SOUTH:
                    theta += L_PI / 2;
                    break;
                case MOVEMENT_DIR_WEST:
                    theta += L_PI;
                    break;
                default:
                    break;
            }

            Eigen::Vector2d d(x, y);
            Eigen::Matrix2d r;
            r << cos(theta), -sin(theta), sin(theta), cos(theta);
            auto rotated = r * d;
            x = rotated.x();
            y = rotated.y();
        }
    } else {
        auto tmp = x;
        x = y;
        y = -tmp;
    }

    this->setGamepadStatus(GamepadStatus::INPUT_RECEIVED);

    rot = rotation_velocity * rot;

    Eigen::Vector2d d(x, y);
    d.normalize();
    x = d.x();
    y = d.y();

    x *= movement_velocity;
    y *= movement_velocity;

    robot_interface::MoveCommand move_command = robot_interface::RobotVelocityControl{{x, y, rot}};

    // logger.info("Send command {}", robot_interface::RobotVelocityControl{{x, y, 1.0f}}.desired_velocity);

    setLocalPlannerDisabledForRobot(robot, true);
    this->robot_interface.updateMoveCommand(robot, move_command);
}  // namespace luhsoccer::luhviz

PerRobotControlData& DataProxy::getPerRobotControlerData(RobotIdentifier selected_robot_id) {
    // get data for selected robot

    auto it = this->control_states_per_robot.find(selected_robot_id);
    if (it == this->control_states_per_robot.end()) {
        // not present -> add id to list
        it = this->control_states_per_robot.emplace(selected_robot_id, PerRobotControlData()).first;
    }
    return it->second;
}

void DataProxy::pollSelections() {
    this->selected_vision_source = this->ssl.getVisionDataSource();
    this->selected_vision_publish_mode = this->ssl.getVisionPublishMode();
    this->selected_gamecontroller_data_source = this->ssl.getGameControllerDataSource();
    this->selected_robot_connection = this->robot_interface.getConnectionType();
    this->selected_simulation_connector = this->sim.getConnector();
}

void DataProxy::loadAllConfigs() {
    // clear because this is called continuesly with a delay
    this->config_params.clear();

    try {
        for (auto& cfg : this->cs.getConfigs()) {
            for (auto& i : cfg->getParams()) {
                switch (i.second->getType()) {
                    case luhsoccer::config_provider::datatypes::Type::INT: {
                        auto& el = dynamic_cast<luhsoccer::config_provider::IntParam>(*i.second);
                        auto list = this->config_params.find(cfg->getConfigName());
                        if (list != this->config_params.end()) {
                            list->second.emplace_back(std::make_unique<ConfigInt>(
                                el, cfg->getConfigName(), el.getGroupName(), el.getKey(), el.getDescription(),
                                el.isWritable(), el.getDefaultValue(), el.getMin(), el.getMax()));
                        } else {
                            std::vector<std::unique_ptr<ConfigParam>> params{};
                            params.emplace_back(std::make_unique<ConfigInt>(
                                el, cfg->getConfigName(), el.getGroupName(), el.getKey(), el.getDescription(),
                                el.isWritable(), el.getDefaultValue(), el.getMin(), el.getMax()));
                            this->config_params.insert(std::make_pair(cfg->getConfigName(), std::move(params)));
                        }
                        break;
                    }
                    case luhsoccer::config_provider::datatypes::Type::BOOL: {
                        auto& el = dynamic_cast<luhsoccer::config_provider::BoolParam>(*i.second);
                        auto list = this->config_params.find(cfg->getConfigName());
                        if (list != this->config_params.end()) {
                            list->second.emplace_back(std::make_unique<ConfigBool>(
                                el, cfg->getConfigName(), el.getGroupName(), el.getKey(), el.getDescription(),
                                el.isWritable(), el.getDefaultValue()));
                        } else {
                            std::vector<std::unique_ptr<ConfigParam>> params{};
                            params.emplace_back(std::make_unique<ConfigBool>(
                                el, cfg->getConfigName(), el.getGroupName(), el.getKey(), el.getDescription(),
                                el.isWritable(), el.getDefaultValue()));
                            this->config_params.insert(std::make_pair(cfg->getConfigName(), std::move(params)));
                        }
                        break;
                    }
                    case luhsoccer::config_provider::datatypes::Type::DOUBLE: {
                        auto& el = dynamic_cast<luhsoccer::config_provider::DoubleParam>(*i.second);
                        auto list = this->config_params.find(cfg->getConfigName());
                        if (list != this->config_params.end()) {
                            list->second.emplace_back(std::make_unique<ConfigDouble>(
                                el, cfg->getConfigName(), el.getGroupName(), el.getKey(), el.getDescription(),
                                el.isWritable(), el.getDefaultValue(), el.getMin(), el.getMax()));
                        } else {
                            std::vector<std::unique_ptr<ConfigParam>> params{};
                            params.emplace_back(std::make_unique<ConfigDouble>(
                                el, cfg->getConfigName(), el.getGroupName(), el.getKey(), el.getDescription(),
                                el.isWritable(), el.getDefaultValue(), el.getMin(), el.getMax()));
                            this->config_params.insert(std::make_pair(cfg->getConfigName(), std::move(params)));
                        }
                        break;
                    }
                    case luhsoccer::config_provider::datatypes::Type::STRING: {
                        auto& el = dynamic_cast<luhsoccer::config_provider::StringParam>(*i.second);
                        auto list = this->config_params.find(cfg->getConfigName());
                        if (list != this->config_params.end()) {
                            list->second.emplace_back(std::make_unique<ConfigString>(
                                el, cfg->getConfigName(), el.getGroupName(), el.getKey(), el.getDescription(),
                                el.isWritable(), el.getDefaultValue()));
                        } else {
                            std::vector<std::unique_ptr<ConfigParam>> params{};
                            params.emplace_back(std::make_unique<ConfigString>(
                                el, cfg->getConfigName(), el.getGroupName(), el.getKey(), el.getDescription(),
                                el.isWritable(), el.getDefaultValue()));
                            this->config_params.insert(std::make_pair(cfg->getConfigName(), std::move(params)));
                        }
                        break;
                    }
                }
            }
        }
    } catch (std::bad_cast& e) {
        logger.error("{}", e.what());
    }

    // sort all params in the map by the group and inside the same group by the key
    auto compare = [](const std::unique_ptr<ConfigParam>& a, const std::unique_ptr<ConfigParam>& b) {
        bool same_group = a->group.compare(b->group) == 0;
        if (same_group) {
            return a->key.compare(b->key) < 0;
        } else {
            return a->group.compare(b->group) < 0;
        }
    };
    for (auto& [config_name, config_vector] : this->getAllConfigParams()) {
        std::sort(config_vector.begin(), config_vector.end(), compare);
    }
}

bool DataProxy::executeScenario(int scenario_index, int repetitions) {
    if (scenario_index < static_cast<int>(scenario::book::BOOK.size())) {
        auto it = scenario::book::BOOK.begin();
        std::advance(it, scenario_index);
        return this->scenario_executor.startScenario(it->second, repetitions);
    }
    logger.warning("the selected scenario with index {} is out of bounds", scenario_index);
    return false;
}

std::string DataProxy::getAvailableScenariosAsString() {
    using namespace std::string_literals;
    auto separator = "\0"s;

    return std::accumulate(scenario::book::BOOK.begin(), scenario::book::BOOK.end(), ""s,
                           [&separator](const std::string& a, const auto& b) { return a + b.first + separator; }) +
           separator;
}

bool DataProxy::setLocalPlannerDisabledForRobot(const RobotIdentifier& robot, bool disabled) {
    auto current_state = this->robot_control.getState(robot);
    if (current_state.has_value()) {
        bool is_disabled = current_state.value() == robot_control::RobotControllerState::OFFLINE;
        if (disabled != is_disabled) {
            // only disable/enable if its not already the case
            return this->robot_control.setControllerActive(robot, !disabled);
        }
    }
    return false;
}

void DataProxy::updateConfigParam(const std::string& config_name, const std::string& key,
                                  std::unique_ptr<ConfigParam>& new_value) {
    try {
        for (auto& cfg : this->cs.getConfigs()) {
            if (cfg->getConfigName().compare(config_name) != 0) continue;

            bool exit = false;
            auto& params = cfg->getParams();  // params map of the current config
            auto it = params.find(key);       // try to find the param with the key

            // cast ConfigParam to correct type and apply it if the types match
            switch (new_value->getType()) {
                case ParamType::INT: {
                    if (it != params.end() && it->second->getType() == config_provider::datatypes::Type::INT) {
                        auto& cs_val = dynamic_cast<luhsoccer::config_provider::IntParam>(*it->second);
                        // if the param is found, apply it and break the loop
                        auto& val = dynamic_cast<ConfigInt&>(*new_value.get());
                        cs_val = val.val;
                        exit = true;
                    }
                    break;
                }
                case ParamType::DOUBLE: {
                    if (it != params.end() && it->second->getType() == config_provider::datatypes::Type::DOUBLE) {
                        auto& cs_val = dynamic_cast<luhsoccer::config_provider::DoubleParam>(*it->second);
                        // if the param is found, apply it and break the loop
                        auto& val = dynamic_cast<ConfigDouble&>(*new_value.get());
                        cs_val = val.val;
                        exit = true;
                    }
                    break;
                }
                case ParamType::BOOL: {
                    if (it != params.end() && it->second->getType() == config_provider::datatypes::Type::BOOL) {
                        auto& cs_val = dynamic_cast<luhsoccer::config_provider::BoolParam>(*it->second);
                        // if the param is found, apply it and break the loop
                        auto& val = dynamic_cast<ConfigBool&>(*new_value.get());
                        cs_val = val.val;
                        exit = true;
                    }
                    break;
                }
                case ParamType::STRING: {
                    if (it != params.end() && it->second->getType() == config_provider::datatypes::Type::STRING) {
                        auto& cs_val = dynamic_cast<luhsoccer::config_provider::StringParam>(*it->second);
                        // if the param is found, apply it and break the loop
                        auto& val = dynamic_cast<ConfigString&>(*new_value.get());
                        cs_val = val.val;
                        exit = true;
                    }
                    break;
                }
                default:
                    logger.warning("problem with saving the config value");
                    break;
            }
            // break loop to save performance if the values was already assigned
            if (exit) break;
        }
    } catch (std::bad_cast& e) {
        logger.error("{}", e.what());
    }
}

int DataProxy::getConfigInt(const std::string& config_name, const std::string& key) {
    const auto& cfg_it = this->config_params.find(config_name);
    try {
        if (cfg_it != this->config_params.end()) {
            const auto& param_it = std::find_if(cfg_it->second.begin(), cfg_it->second.end(),
                                                [key](const auto& p) { return p->key.compare(key) == 0; });
            if (param_it != cfg_it->second.end()) {
                auto& param = dynamic_cast<ConfigInt&>(*param_it->get());
                return param.val;
            }
        }
    } catch (std::bad_cast& e) {
        logger.error("{}", e.what());
    }
    logger.warning("could not find ConfigInt {} in config {}", key, config_name);
    return 0;
}

float DataProxy::getConfigDouble(const std::string& config_name, const std::string& key) {
    const auto& cfg_it = this->config_params.find(config_name);
    try {
        if (cfg_it != this->config_params.end()) {
            const auto& param_it = std::find_if(cfg_it->second.begin(), cfg_it->second.end(),
                                                [key](const auto& p) { return p->key.compare(key) == 0; });
            if (param_it != cfg_it->second.end()) {
                auto& param = dynamic_cast<ConfigDouble&>(*param_it->get());
                return static_cast<float>(param.val);
            }
        }
    } catch (std::bad_cast& e) {
        logger.error("{}", e.what());
    }
    logger.warning("could not find ConfigDouble {} in config {}", key, config_name);
    return 0.0;
}

bool DataProxy::getConfigBool(const std::string& config_name, const std::string& key) {
    const auto& cfg_it = this->config_params.find(config_name);
    try {
        if (cfg_it != this->config_params.end()) {
            const auto& param_it = std::find_if(cfg_it->second.begin(), cfg_it->second.end(),
                                                [key](const auto& p) { return p->key.compare(key) == 0; });
            if (param_it != cfg_it->second.end()) {
                auto& param = dynamic_cast<ConfigBool&>(*param_it->get());
                return param.val;
            }
        }
    } catch (std::bad_cast& e) {
        logger.error("{}", e.what());
    }
    logger.warning("could not find ConfigBool {} in config {}", key, config_name);
    return false;
}

std::string DataProxy::getConfigString(const std::string& config_name, const std::string& key) {
    const auto& cfg_it = this->config_params.find(config_name);
    try {
        if (cfg_it != this->config_params.end()) {
            const auto& param_it = std::find_if(cfg_it->second.begin(), cfg_it->second.end(),
                                                [key](const auto& p) { return p->key.compare(key) == 0; });
            if (param_it != cfg_it->second.end()) {
                auto& param = dynamic_cast<ConfigString&>(*param_it->get());
                return param.val;
            }
        }
    } catch (std::bad_cast& e) {
        logger.error("{}", e.what());
    }
    logger.warning("could not find ConfigString {} in config {}", key, config_name);
    return "";
}

void DataProxy::setConfigInt(const std::string& key, int value) {
    try {
        for (auto& cfg : this->config_params) {
            const auto& it = std::find_if(cfg.second.begin(), cfg.second.end(),
                                          [key](const auto& p) { return p->key.compare(key) == 0; });
            if (it != cfg.second.end()) {
                auto& param = dynamic_cast<ConfigInt&>(*it->get());
                param.val = value;
                param.synced_with_config_provider = false;
            }
        }
    } catch (std::bad_cast& e) {
        logger.error("{}", e.what());
    }
}

void DataProxy::setConfigDouble(const std::string& key, double value) {
    try {
        for (auto& cfg : this->config_params) {
            const auto& it = std::find_if(cfg.second.begin(), cfg.second.end(),
                                          [key](const auto& p) { return p->key.compare(key) == 0; });
            if (it != cfg.second.end()) {
                auto& param = dynamic_cast<ConfigDouble&>(*it->get());
                param.val = value;
                param.synced_with_config_provider = false;
            }
        }
    } catch (std::bad_cast& e) {
        logger.error("{}", e.what());
    }
}

void DataProxy::setConfigBool(const std::string& key, bool value) {
    try {
        for (auto& cfg : this->config_params) {
            const auto& it = std::find_if(cfg.second.begin(), cfg.second.end(),
                                          [key](const auto& p) { return p->key.compare(key) == 0; });
            if (it != cfg.second.end()) {
                auto& param = dynamic_cast<ConfigBool&>(*it->get());
                param.val = value;
                param.synced_with_config_provider = false;
            }
        }
    } catch (std::bad_cast& e) {
        logger.error("{}", e.what());
    }
}

void DataProxy::setConfigString(const std::string& key, const std::string& value) {
    try {
        for (auto& cfg : this->config_params) {
            const auto& it = std::find_if(cfg.second.begin(), cfg.second.end(),
                                          [key](const auto& p) { return p->key.compare(key) == 0; });
            if (it != cfg.second.end()) {
                auto& param = dynamic_cast<ConfigString&>(*it->get());
                param.val = value;
                param.synced_with_config_provider = false;
            }
        }
    } catch (std::bad_cast& e) {
        logger.error("{}", e.what());
    }
}

bool DataProxy::getChipperOn() { return this->chipper_on; }

float& DataProxy::getKickVelocity() { return this->kick_velocity; }

}  // namespace luhsoccer::luhviz