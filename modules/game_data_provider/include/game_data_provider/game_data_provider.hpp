#pragma once

#include <map>
#include <condition_variable>
#include "common_types.hpp"
#include <optional>
#include <shared_mutex>
#include <vector>
#include "transform/transform.hpp"
#include "logger/logger.hpp"
#include "transform/game_state.hpp"
#include "game_data_provider/team_info.hpp"
#include "module.hpp"
#include "marker_service/marker.hpp"

namespace luhsoccer::ssl_interface {
class SSLInterface;
struct SSLVisionData;
struct SSLFieldData;
struct SSLGameControllerData;
struct SSLRobotInfo;
struct SSLBallInfo;
struct SSLTeamInfo;
}  // namespace luhsoccer::ssl_interface

namespace luhsoccer::robot_interface {
class RobotInterface;
struct RobotCommand;
struct RobotFeedback;
}  // namespace luhsoccer::robot_interface

namespace luhsoccer::marker {
class MarkerService;
}

namespace luhsoccer::observer {
class Observer;
}

namespace luhsoccer::game_data_provider {

class RobotDataFilter;
class BallFilter;
class DataProcessor;

using GameState = transform::GameState;

/**
 * @brief Game Data Provider provides Game Data from sll interface and robot interface
 *
 */
class GameDataProvider : public BaguetteModule {
   public:
    GameDataProvider(ssl_interface::SSLInterface& ssl_interface, robot_interface::RobotInterface& robot_interface,
                     marker::MarkerService& ms);
    virtual ~GameDataProvider();
    GameDataProvider(const GameDataProvider& provider) = delete;
    GameDataProvider(const GameDataProvider&& provider) = delete;
    GameDataProvider& operator=(const GameDataProvider& provider) = delete;
    GameDataProvider& operator=(const GameDataProvider&& provider) = delete;

    void setup() override;

    // The loop method of a BaguetteModule
    void loop(std::atomic_bool& should_run) override;

    void stop() override;

    /**
     * @brief Returns a reference to the WorldModel
     *
     * @return  std::shared_ptr<const transform::WorldModel>
     */
    [[nodiscard]] std::shared_ptr<const transform::WorldModel> getWorldModel() const { return this->world_model; }
    [[nodiscard]] std::shared_ptr<const observer::Observer> getObserver() const { return this->observer; }

    [[nodiscard]] constexpr std::string_view moduleName() override { return "game_data_provider"; }

    time::TimePoint getVisionDataTimestamp() const;
    time::TimePoint getGameControllerDataTimestamp() const;

    TeamInfo getAllyTeamInfo() const;
    TeamInfo getEnemyTeamInfo() const;

    RobotIdentifier getGoalie() const;
    RobotIdentifier getEnemyGoalie() const;

    std::optional<Eigen::Vector2d> getSpecialKickPosition();
    std::optional<time::TimePoint> getSpecialKickTime();

   private:
    void onNewVisionData(const ssl_interface::SSLVisionData& data);
    void onNewFieldData(const ssl_interface::SSLFieldData& data);
    void onNewRobotCommand(const std::pair<uint32_t, robot_interface::RobotCommand>& data);
    void onNewGameControllerData(const ssl_interface::SSLGameControllerData& data);
    void onNewRobotFeedback(std::pair<RobotIdentifier, robot_interface::RobotFeedback> data);
    void publishRobotToWorldModel(const ssl_interface::SSLRobotInfo& info, const RobotIdentifier& id,
                                  const std::string& appendix = "");
    void publishRobotToWorldModel(const Eigen::Affine2d& affine, const RobotIdentifier& id,
                                  const std::string& appendix = "");
    void publishBall(const ssl_interface::SSLBallInfo& info);
    void publishRobotData(const RobotIdentifier& handle);
    TeamInfo processTeamInfo(const ssl_interface::SSLTeamInfo& info);

    static void startObserverThread(const GameDataProvider& gdp);

    void removeMissingRobots();

    template <enum TeamColor color>
    [[nodiscard]] RobotIdentifier createRIDFromId(unsigned int robot_id) const {
        std::shared_lock lock{this->team_color_mutex};
        if (color == this->our_team_color) {
            return {robot_id, Team::ALLY};
        } else {
            return {robot_id, Team::ENEMY};
        }
    }

    GameState nextGameState(const ssl_interface::SSLGameControllerData& data);
    void updateGameState();

   private:
    const inline static std::string BALL_FRAME{"ball"};
    const inline static std::string BALL_PLACEMENT_POSITION_FRAME{"ball_placement_position"};
    // TODO change per config or by values from game controller
    constexpr inline static size_t MAX_ACTIVE_ROBOTS_PER_TEAM = 6;
    constexpr inline static time::Duration MAX_ROBOT_MISSING_TIME = time::Duration(0.2);
    const transform::Position ball_position{BALL_FRAME};

    ssl_interface::SSLInterface& ssl_interface;
    robot_interface::RobotInterface& robot_interface;
    std::shared_ptr<std::function<void(const std::pair<RobotIdentifier, robot_interface::RobotFeedback>&)>>
        feedback_callback;

    std::shared_ptr<std::function<void(const std::pair<uint32_t, robot_interface::RobotCommand>&)>> command_callback;
    marker::MarkerService& ms;
    std::shared_ptr<transform::WorldModel> world_model;

    TeamColor our_team_color{TeamColor::BLUE};  // TODO change by config??
    RobotIdentifier goalie{0, Team::ALLY};
    RobotIdentifier enemy_goalie{0, Team::ENEMY};

    mutable std::shared_mutex team_color_mutex;

    mutable std::shared_mutex vision_data_mutex;
    time::TimePoint vision_data_timestamp;

    mutable std::shared_mutex gamecontroller_data_mutex;

    /**
     * @brief the Continuous Observer
     */
    std::shared_ptr<observer::Observer> observer;

    time::TimePoint gamecontroller_data_timestamp;

    mutable std::mutex game_state_mutex;
    GameState current_state{GameState::HALT};
    TeamInfo ally_team_info{};
    TeamInfo enemy_team_info{};

    std::unordered_map<RobotIdentifier, std::unique_ptr<RobotDataFilter>> robot_data_filters;
    time::TimePoint last_filter_param_update{};
    time::Duration filter_update_time{1.0};
    std::unique_ptr<BallFilter> ball_filter;

    std::atomic_bool should_run{true};
    std::thread observer_thread;
    mutable std::condition_variable observer_trigger;
    mutable std::mutex observer_mtx;

    std::unique_ptr<DataProcessor> data_processor;

    std::mutex robot_infos_mutex;
    std::unordered_map<RobotIdentifier, marker::RobotInfo> robot_infos;

    marker::LinePlot frequency_plot;
    marker::LinePlot ball_velocity_plot;

    // The ball position when a kickoff, free kick or penalty kick was issued
    std::optional<Eigen::Vector2d> special_kick_ball_position;
    std::optional<time::TimePoint> special_kick_timestamp;

    logger::Logger logger{"game_data_provider"};
};
}  // namespace luhsoccer::game_data_provider