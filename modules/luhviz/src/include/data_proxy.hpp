#pragma once
#include <imgui.h>
#include "scenario/scenario_executor.hpp"
#include "scenario/scenario_book.hpp"
#include "ssl_interface/ssl_interface.hpp"
#include "config/config_store.hpp"
#include "config_provider/config_store_main.hpp"
#include <glm/glm.hpp>
#include <utility>
#include "data_structs.hpp"
#include "simulation_interface/simulation_interface.hpp"
#include "local_planner/local_planner_module.hpp"
#include "skill_books/bod_skill_book.hpp"
#include "robot_interface/robot_interface.hpp"
#include "game_data_provider/game_data_provider.hpp"
#include "marker_service/marker_impl.hpp"
#include "marker_service/marker_2d_impl.hpp"

namespace luhsoccer::luhviz {

enum class GamepadControls { BUTTON_KICKER, BUTTON_DRIBBLER, BUTTON_VOLTAGE_UP, BUTTON_VOLTAGE_DOWN };
enum class GamepadStatus { DISSCONECTED, CONNECTED, INPUT_RECEIVED };

struct PerRobotControlData {
    robot_interface::DribblerMode last_dribbler = robot_interface::DribblerMode::OFF;
    std::optional<double> last_kick_time;
    float last_x = 0.0f;
    float last_y = 0.0f;
    float last_rot = 0.0f;
};

struct PerControllerData {
    float last_x = 0.0f;
    float last_y = 0.0f;
    float last_rot = 0.0f;
    int gamepad_dribbler_buttonstate = 0;
    int gamepad_voltage_up_buttonstate = 0;
    int gamepad_voltage_down_buttonstate = 0;
    std::optional<RobotIdentifier> robot_id;
};

class DataProxy {
   public:
    DataProxy(ssl_interface::SSLInterface& ssl, simulation_interface::SimulationInterface& sim,
              local_planner::LocalPlannerModule& local_planner, skills::BodSkillBook& skill_book,
              robot_interface::RobotInterface& robot_interface, game_data_provider::GameDataProvider& gdp,
              scenario::ScenarioExecutor& scenario_executor);

    std::map<size_t, RobotData> robot_data{};
    BallData ball_data{};

    ImVec4 accent_text_color;

    void setVisionSource(const std::string& src);
    std::string getVisionSource();
    void setRobotConnection(const std::string& src);
    std::string getRobotConnection();
    void setSimulationConnection(const std::string& src);
    std::string getSimulationConnection();
    void setGameControllerDataSource(const std::string& src);
    std::string getGameControllerDataSource();
    void setVisionPublishMode(const std::string& src);
    std::string getVisionPublishMode();

    std::unordered_map<std::string, std::vector<std::unique_ptr<ConfigParam>>>& getAllConfigParams() {
        return this->config_params;
    }

    /**
     * @brief saves the int param in config_provider
     *
     */
    void updateConfigParam(const std::string& key, std::unique_ptr<ConfigParam>& new_value);

    /**
     * @brief saves all configs permantently
     *
     */
    void saveAllConfigs() { this->cs.saveAll(); }

    /**
     * @brief saves all configs permantently
     *
     */
    void saveLuhvizConfig() { this->cs.luhviz_internal_config.save(); }

    /**
     * @brief forwards the ball teleport command to the simulation
     *
     * @param target
     */
    void teleportBall(const Eigen::Affine2d& target, const Eigen::Vector3d& velocity = {0., 0., 0.});

    /**
     * @brief Get the Current Ball Pos from world model
     *
     * @return std::optional<Eigen::Affine2d>
     */
    std::optional<Eigen::Affine2d> getCurrentBallPos();

    /**
     * @brief forwards the robot teleport command to the simulation
     *
     * @param robot_id
     * @param team_color
     * @param target
     */
    void teleportRobot(size_t robot_id, TeamColor team_color, const Eigen::Affine2d& target, bool present);

    /**
     * @brief sends the current skill and taskdata to the local planner
     *
     */
    bool sendSkillsToRobots();

    /**
     * @brief sends a cancel command to cancel the current task of the given robot to the local planner
     *
     * @return true success
     * @return false was not executing a task
     */
    bool cancelSkills();

    /**
     * @brief cancels the robots current task
     *
     * @param robot
     * @return true
     * @return false
     */
    bool cancelSkill(RobotIdentifier& robot);

    /**
     * @brief get all skills as string
     *
     * @return std::string
     */
    std::string getSkillNames() {
        if (!skills_loaded) {
            loadAvailableSkills();
        }
        return this->available_skills_names;
    }

    /**
     * @brief sets the selected skill if it has changed
     *
     * @param skill_index
     * @return std::optional<local_planner::Skill*> the selected skill object
     */
    std::optional<local_planner::Skill*> setSelectedSkill(std::optional<size_t> skill_index);

    /**
     * @brief sets the selected skill if it has changed
     *
     * @param skill_index
     * @return std::optional<local_planner::Skill*> the selected skill object
     */
    std::optional<local_planner::Skill*> setSelectedSkill2(std::optional<size_t> skill_index);

    /**
     * @brief returns the count of remaining points to choose for the taskData object
     *
     * @return size_t
     */
    size_t getRemainingPointsToChoose();

    /**
     * @brief returns the count of remaining points to choose for the taskData object
     *
     * @return size_t
     */
    size_t getRemainingPointsToChoose2();

    /**
     * @brief Get the Count Rel Robots Selected object
     *
     * @return int
     */
    int getCountRelRobotsSelected();

    /**
     * @brief Get the Count Rel Robots Selected object
     *
     * @return int
     */
    int getCountRelRobotsSelected2();

    /**
     * @brief Create a Task Data object
     *
     * @param robot_id
     */
    void createTaskData(RobotIdentifier robot_id) { this->task_data = local_planner::TaskData{robot_id}; }

    /**
     * @brief Create a Task Data object
     *
     * @param robot_id
     */
    void createTaskData2(RobotIdentifier robot_id) { this->task_data2 = local_planner::TaskData{robot_id}; }

    /**
     * @brief set the related robots of the task data object
     *
     * @param related_robots
     */
    void setTDRobots(std::vector<std::optional<RobotIdentifier>> related_robots) {
        this->td_related_robots = std::move(related_robots);
    }

    /**
     * @brief set the related robots of the task data object
     *
     * @param related_robots
     */
    void setTDRobots2(std::vector<std::optional<RobotIdentifier>> related_robots) {
        this->td_related_robots2 = std::move(related_robots);
    }

    /**
     * @brief set the next empty related robots of the task data object to this robot
     *
     * @param related_robot
     */
    void updateNextTDRelatedRobot(const RobotIdentifier& related_robot);

    /**
     * @brief get defined task data points
     *
     * @return std::vector<glm::dvec3>
     */
    std::vector<glm::dvec3> getTDPoints() { return this->td_positions; }

    /**
     * @brief get defined task data points
     *
     * @return std::vector<glm::dvec3>
     */
    std::vector<glm::dvec3> getTDPoints2() { return this->td_positions2; }

    /**
     * @brief add point to task data object
     *
     * @param position
     */
    void addTDPoint(const glm::dvec3& position) {
        if (this->task_data.has_value() && getRemainingPointsToChoose() > 0) {
            // only save position if taskdata is valid
            this->td_positions.emplace_back(position);
            this->last_td_point_index = this->td_positions.size() - 1;
        } else {
            LOG_WARNING(logger, "added more than the required points to taskdata");
        }
    }

    /**
     * @brief add point to task data object
     *
     * @param position
     */
    void addTDPoint2(const glm::dvec3& position) {
        if (this->task_data2.has_value() && getRemainingPointsToChoose2() > 0) {
            // only save position if taskdata is valid
            this->td_positions2.emplace_back(position);
            this->last_td_point_index2 = this->td_positions2.size() - 1;
        } else {
            LOG_WARNING(logger, "added more than the required points to taskdata 2");
        }
    }

    /**
     * @brief removes the newest point from the list
     *
     */
    void removeNewestTDPoint() {
        if (this->last_td_point_index.has_value()) {
            this->td_positions.erase(this->td_positions.begin() + this->last_td_point_index.value());
            this->last_td_point_index = std::nullopt;
            this->manipulation_mode = ManipulationMode::ADD_POINT;
        }
    }

    /**
     * @brief set point of task data object
     *
     * @param index
     * @param new_point
     */
    void updateTDPoint(size_t index, const glm::dvec3 new_point) {
        if (index < this->td_positions.size()) {
            this->td_positions[index] = new_point;
        }
    }

    /**
     * @brief set point of task data object
     *
     * @param index
     * @param new_point
     */
    void updateTDPoint2(size_t index, const glm::dvec3 new_point) {
        if (index < this->td_positions2.size()) {
            this->td_positions2[index] = new_point;
        }
    }

    /**
     * @brief set task data bool values
     *
     * @param required_bools
     */
    void setTDBools(std::vector<bool> required_bools) {
        if (this->task_data.has_value()) {
            this->task_data.value().required_bools = std::move(required_bools);
        }
    }

    /**
     * @brief set task data bool values
     *
     * @param required_bools
     */
    void setTDBools2(std::vector<bool> required_bools) {
        if (this->task_data2.has_value()) {
            this->task_data2.value().required_bools = std::move(required_bools);
        }
    }

    /**
     * @brief set task data int values
     *
     * @param required_ints
     */
    void setTDInts(std::vector<int> required_ints) {
        if (this->task_data.has_value()) {
            this->task_data.value().required_ints = std::move(required_ints);
        }
    }

    /**
     * @brief set task data int values
     *
     * @param required_ints
     */
    void setTDInts2(std::vector<int> required_ints) {
        if (this->task_data2.has_value()) {
            this->task_data2.value().required_ints = std::move(required_ints);
        }
    }

    /**
     * @brief set task data double values
     *
     * @param required_doubles
     */
    void setTDDoubles(std::vector<double> required_doubles) {
        if (this->task_data.has_value()) {
            this->task_data.value().required_doubles = std::move(required_doubles);
        }
    }

    /**
     * @brief set task data double values
     *
     * @param required_doubles
     */
    void setTDDoubles2(std::vector<double> required_doubles) {
        if (this->task_data2.has_value()) {
            this->task_data2.value().required_doubles = std::move(required_doubles);
        }
    }

    /**
     * @brief set task data string values
     *
     * @param required_strings
     */
    void setTDStrings(std::vector<std::string> required_strings) {
        if (this->task_data.has_value()) {
            this->task_data.value().required_strings = std::move(required_strings);
        }
    }

    /**
     * @brief set task data string values
     *
     * @param required_strings
     */
    void setTDStrings2(std::vector<std::string> required_strings) {
        if (this->task_data2.has_value()) {
            this->task_data2.value().required_strings = std::move(required_strings);
        }
    }

    /**
     * @brief Get the selected robot
     *
     * @return std::optional<RobotIdentifier>&
     */
    std::optional<RobotIdentifier>& getSelectedRobot() { return this->selected_robot; }

    /**
     * @brief Get the selected robot 2
     *
     * @return std::optional<RobotIdentifier>&
     */
    std::optional<RobotIdentifier>& getSelectedRobot2() { return this->selected_robot2; }

    /**
     * @brief if the robot should be present on the game field
     *
     * @return true
     * @return false
     */
    bool isRobotPresent() { return this->robot_present; }

    /**
     * @brief set the robot present value
     *
     * @param present
     */
    void setRobotPresent(bool present) { this->robot_present = present; }

    /**
     * @brief Get the Controlled Robot
     *
     * @return std::optional<RobotIdentifier>&
     */
    std::map<size_t, RobotIdentifier>& getControlledRobots() { return this->controlled_robots; }

    /**
     * @brief Get the Controler Data
     *
     * @return std::optional<RobotIdentifier>&
     */
    std::map<size_t, PerControllerData>& getControllerData() { return this->controller_data; }

    /**
     * @brief Clear all robot identifyers from controller data
     *
     * @return std::optional<RobotIdentifier>&
     */
    void clearControllerRobotidentifyer() {
        for (auto& data : this->controller_data) {
            data.second.robot_id.reset();
        }
    }

    /**
     * @brief Get all selected skill related robots
     *
     * @return std::vector<std::optional<RobotIdentifier>>&
     */
    std::vector<std::optional<RobotIdentifier>>& getSelectedRelatedRobots() { return this->td_related_robots; }

    /**
     * @brief Get all selected skill related robots
     *
     * @return std::vector<std::optional<RobotIdentifier>>&
     */
    std::vector<std::optional<RobotIdentifier>>& getSelectedRelatedRobots2() { return this->td_related_robots2; }

    /**
     * @brief get all robots
     *
     * @return std::vector<RobotIdentifier>
     */
    std::vector<RobotIdentifier> getPossibleRobots() { return this->gdp.getWorldModel()->getPossibleRobots(); }

    /**
     * @brief get all robots
     *
     * @return std::vector<RobotIdentifier>
     */
    std::vector<RobotIdentifier> getVisibleRobots() { return this->gdp.getWorldModel()->getVisibleRobots(); }

    /**
     * @brief get all robots
     *
     * @return std::vector<RobotIdentifier>
     */
    std::vector<RobotIdentifier> getRobotsWithEmptyId() {
        auto v = this->gdp.getWorldModel()->getPossibleRobots();
        v.emplace(v.begin(), EMPTY_IDENTIFIER);
        return v;
    }

    /**
     * @brief Get the Robot Marker object
     *
     * @return std::unordered_map<size_t, marker::Robot>
     */
    std::unordered_map<size_t, marker::MarkerImpl>& getRobotMarkers() { return this->robot_markers; }

    /**
     * @brief get the currently active manipulation mode
     *
     * @return ManipulationMode&
     */
    ManipulationMode& getManipulationMode() { return this->manipulation_mode; }

    /**
     * @brief reset the task data object and all inputs to default
     *
     */
    void clearTaskData();

    /**
     * @brief return true if the second skill is enabled
     *
     * @return true
     * @return false
     */
    bool& getSecondSkillEnabled() { return this->second_skill_enabled; }

    /**
     * @brief Get the currrent game state
     *
     */
    std::optional<transform::GameState> getCurrentGameState() { return this->game_data.game_state; }

    /**
     * @brief returns the ally team info
     *
     * @return game_data_provider::TeamInfo
     */
    std::optional<game_data_provider::TeamInfo> getAllyTeamInfo() { return this->game_data.ally_info; }

    /**
     * @brief returns the enemy team info
     *
     * @return game_data_provider::TeamInfo
     */
    std::optional<game_data_provider::TeamInfo> getEnemyTeamInfo() { return this->game_data.enemy_info; }

    /**
     * @brief returns a reference to the Gamepad status
     *
     * @return GamepadStatus&
     */
    GamepadStatus& getGamepadStatus();

    /**
     * @brief sets the gamepad status
     *
     * @param GamepadStatus status to set
     *
     */
    void setGamepadStatus(GamepadStatus status) { this->gamepad_status = status; };

    /**
     * @brief returns the controller count
     *
     * @return GamepadStatus&
     */
    size_t getGamepadCount() {
        if (!this->controller_data.empty())
            return this->controller_data.size() - 1;
        else
            return 0;
    };

    /**
     * @brief handles button input for connected gamepads
     *
     * @param input kicker or dribbler button
     * @param button_state is 0 if released, otherwise button is pressed
     * @param id id of controller
     * @param current_time current time frame in glfw
     */
    void gamepadButtonInput(GamepadControls input, int button_state, size_t id, double current_time);

    /**
     * @brief handles axes input for connected gamepads
     *
     * @param x horizontal axes
     * @param y vertical axes
     * @param rot rotational input
     */
    void gamepadAxesInput(float x, float y, float rot, size_t id, bool global_movement);

    /**
     * @brief sends all dribbler states
     * @brief sends all states that need to be send periodically (Dribbler, Zero-Kick)
     */
    void publishRobotData(double current_time);

    /**
     * @brief update loop
     *
     */
    void update();

    /**
     * @brief updates the game info data from the gdp/wm
     *
     */
    void updateGameInfoData();

    /**
     * @brief polls selection info from interfaces to update GUI
     *
     *
     */
    void pollSelections();

    /**
     * @brief Set the Log File Path for the game log to play
     *
     * @param path
     */
    void setLogFilePath(const std::string& path) { this->logfile_path = path; }
    std::optional<std::string> getLogFilePath() { return this->logfile_path; }

    /**
     * @brief Get the Config Int
     *
     * @param config_name
     * @param key
     * @return int
     */
    int getConfigInt(const std::string& config_name, const std::string& key);

    /**
     * @brief Get the Config Double
     *
     * @param config_name
     * @param key
     * @return float
     */
    float getConfigDouble(const std::string& config_name, const std::string& key);

    /**
     * @brief Get the Config Bool
     *
     * @param config_name
     * @param key
     * @return true
     * @return false
     */
    bool getConfigBool(const std::string& config_name, const std::string& key);

    /**
     * @brief Get the Config String
     *
     * @param config_name
     * @param key
     * @return std::string
     */
    std::string getConfigString(const std::string& config_name, const std::string& key);

    /**
     * @brief Set the Config Int
     *
     * @param key
     * @param value
     */
    void setConfigInt(const std::string& key, int value);

    /**
     * @brief Set the Config Double
     *
     * @param key
     * @param value
     */
    void setConfigDouble(const std::string& key, double value);

    /**
     * @brief Set the Config Bool
     *
     * @param key
     * @param value
     */
    void setConfigBool(const std::string& key, bool value);

    /**
     * @brief Set the Config String
     *
     * @param key
     * @param value
     */
    void setConfigString(const std::string& key, const std::string& value);

    /**
     * @brief loads all config values from the config_provider at startup into custom structs
     *
     */
    void loadAllConfigs();

    /**
     * @brief sends a start command to the scenario executor
     *
     * @param scenario the scenario which should be executed
     * @return true
     * @return false
     */
    bool executeScenario(int scenario_index, int repetitions);

    /**
     * @brief Get the Available Scenarios As a imgui string
     *
     * @return std::string
     */
    static std::string getAvailableScenariosAsString();

    /**
     * @brief Get the Kick Voltage
     *
     * @return int
     */
    int getKickVoltage() const;

    /**
     * @brief Get the Movement Velocity of the controlled robot
     *
     * @return float&
     */
    float& getMovementVelocity();

    /**
     * @brief Get the Rotation Velocity of the controlled robot
     *
     * @return float&
     */
    float& getRotationVelocity();

    /**
     * @brief Set the Local Planner disabled of robot
     *
     * @param robot
     * @param disabled
     * @return true
     * @return false
     */
    bool setLocalPlannerDisabledForRobot(const RobotIdentifier& robot, bool disabled);

    /**
     * @brief Get the Ball Velocity from the real worldmodel
     *
     * @return ImVec2
     */
    ImVec2 getBallVelocity() {
        auto vel = this->gdp.getWorldModel()->getVelocity(this->gdp.getWorldModel()->getBallFrame());
        if (vel.has_value()) {
            return ImVec2{static_cast<float>(vel.value().velocity.x()), static_cast<float>(vel.value().velocity.y())};
        }
        return {0, 0};
    }

    /**
     * @brief Get the Game Controller Running status
     *
     * @return true gc is running
     * @return false gc is not running
     */
    bool& getGameControllerRunning() { return this->game_controller_running; }

   private:
    logger::Logger logger{"luhviz/data_proxy"};
    ssl_interface::SSLInterface& ssl;
    robot_interface::RobotInterface& robot_interface;
    config_provider::ConfigStore& cs = config_provider::ConfigProvider::getConfigStore();
    simulation_interface::SimulationInterface& sim;
    local_planner::LocalPlannerModule& local_planner;
    skills::BodSkillBook& skill_book;

    // data
    ssl_interface::VisionDataSource selected_vision_source{};
    ssl_interface::VisionPublishMode selected_vision_publish_mode{};
    ssl_interface::GameControllerDataSource selected_gamecontroller_data_source{};
    robot_interface::RobotConnection selected_robot_connection{};
    simulation_interface::SimulationConnectorType selected_simulation_connector{};
    game_data_provider::GameDataProvider& gdp;
    scenario::ScenarioExecutor& scenario_executor;

    std::unordered_map<size_t, marker::MarkerImpl> robot_markers{};
    std::optional<RobotIdentifier> selected_robot;
    std::optional<RobotIdentifier> selected_robot2;

    std::map<size_t, RobotIdentifier> controlled_robots{};
    std::map<size_t, PerControllerData> controller_data{};
    ManipulationMode manipulation_mode{ManipulationMode::SELECT};

    bool dribbler_high{false};

    // config provider data
    std::unordered_map<std::string, std::vector<std::unique_ptr<ConfigParam>>> config_params{};

    // local planner / skills
    std::vector<local_planner::Skill> available_skills{};
    std::string available_skills_names{""};
    bool skills_loaded{false};
    std::optional<local_planner::Skill*> selected_skill{};
    std::optional<local_planner::Skill*> selected_skill2{};
    std::optional<local_planner::TaskData> task_data{std::nullopt};
    std::optional<local_planner::TaskData> task_data2{std::nullopt};
    std::vector<glm::dvec3> td_positions{};
    std::vector<glm::dvec3> td_positions2{};
    std::vector<std::optional<RobotIdentifier>> td_related_robots{};
    std::vector<std::optional<RobotIdentifier>> td_related_robots2{};
    std::optional<int> last_td_point_index{std::nullopt};
    std::optional<int> last_td_point_index2{std::nullopt};
    bool robot_present{true};
    bool second_skill_enabled{false};

    // controller
    float kick_velocity = 5.0f;
    int cap_voltage = 0;
    const int cap_voltage_steps = 10;
    const int max_cap_voltage = 200;
    // time until zero-kick is init
    double zero_kick_time = 0.5;

    // game infos
    GameDataDisplay game_data{};
    std::chrono::time_point<std::chrono::steady_clock> last_game_info_update_time{std::chrono::steady_clock::now()};

    std::map<RobotIdentifier, PerRobotControlData> control_states_per_robot{};

    robot_interface::DribblerMode last_dribbler = robot_interface::DribblerMode::OFF;
    float last_x = 0.0f;
    float last_y = 0.0f;
    float last_rot = 0.0f;
    float movement_velocity = 1.0f;
    float rotation_velocity = 1.0f;
    GamepadStatus gamepad_status = GamepadStatus::DISSCONECTED;
    int gamepad_dribbler_buttonstate = 0;
    int gamepad_voltage_up_buttonstate = 0;
    int gamepad_voltage_down_buttonstate = 0;

    PerRobotControlData& getPerRobotControlerData(RobotIdentifier selected_robot_id);

    // SSL Log Player
    std::optional<std::string> logfile_path{};

    // Game Controller window
    bool game_controller_running = false;

    /**
     * @brief calls the local planner to check if TaskData for the Selected Skill is valid
     *
     * @return true data is valid
     * @return false
     */
    bool checkTaskData();

    /**
     * @brief calls the local planner to check if TaskData for the Selected Skill is valid
     *
     * @return true data is valid
     * @return false
     */
    bool checkTaskData2();

    /**
     * @brief loads all available skills at startup
     *
     */
    void loadAvailableSkills();
};
}  // namespace luhsoccer::luhviz