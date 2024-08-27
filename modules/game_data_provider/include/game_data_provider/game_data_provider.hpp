#pragma once

#include "core/common_types.hpp"
#include <optional>
#include <shared_mutex>
#include "logger/logger.hpp"
#include "transform/game_state.hpp"
#include "game_data_provider/team_info.hpp"
#include "core/module.hpp"
#include "marker_service/marker.hpp"
#include "game_data_provider_events.hpp"

namespace luhsoccer::ssl_interface {
enum class SSLStage;
struct SSLVisionData;
struct SSLFieldData;
struct SSLGameControllerData;
struct SSLRobotInfo;
struct SSLBallInfo;
struct SSLTeamInfo;
}  // namespace luhsoccer::ssl_interface

namespace luhsoccer::robot_interface {
struct RobotCommand;
struct RobotFeedback;
class RobotCommandSendEvent;
class RobotFeedbackReceivedEvent;
}  // namespace luhsoccer::robot_interface

namespace luhsoccer::vision_processor {
struct ProcessedVisionData;
}

namespace luhsoccer::marker {
class MarkerService;
}

namespace luhsoccer::observer {
class Observer;
}

namespace luhsoccer::event_system {
class EventSystem;
}

namespace luhsoccer::game_data_provider {

using GameState = transform::GameState;

class RobotDataFilter;
class DataProcessor;
class BallFilter;

/**
 * @brief Game Data Provider provides Game Data from sll interface and robot interface
 *
 */
class GameDataProvider : public BaguetteModule {
   public:
    GameDataProvider(marker::MarkerService& ms, event_system::EventSystem& system);
    virtual ~GameDataProvider();
    GameDataProvider(const GameDataProvider& provider) = delete;
    GameDataProvider(const GameDataProvider&& provider) = delete;
    GameDataProvider& operator=(const GameDataProvider& provider) = delete;
    GameDataProvider& operator=(const GameDataProvider&& provider) = delete;

    void setup() override;
    void setup(event_system::EventSystem& system) override;

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

    [[nodiscard]] ssl_interface::SSLStage getGameStage() const;

    TeamInfo getAllyTeamInfo() const;
    TeamInfo getEnemyTeamInfo() const;

    RobotIdentifier getGoalie() const;
    RobotIdentifier getEnemyGoalie() const;

    std::optional<Eigen::Vector2d> getSpecialKickPosition();
    std::optional<time::TimePoint> getSpecialKickTime();

   private:
    void onNewProcessedVisionData(const vision_processor::ProcessedVisionData& data);
    void onNewFieldData(const ssl_interface::SSLFieldData& data);
    void onNewRobotCommand(const robot_interface::RobotCommandSendEvent& data);
    void onNewGameControllerData(const ssl_interface::SSLGameControllerData& data);
    void onNewRobotFeedback(const robot_interface::RobotFeedbackReceivedEvent& feedback);
    void publishRobotToWorldModel(const ssl_interface::SSLRobotInfo& info, const RobotIdentifier& id,
                                  const std::string& appendix = "");
    void publishRobotToWorldModel(const Eigen::Affine2d& affine, const RobotIdentifier& id,
                                  const std::string& appendix = "");
    void publishRobotData(const RobotIdentifier& handle);
    TeamInfo processTeamInfo(const ssl_interface::SSLTeamInfo& info);

    void updateObserver();

    void removeMissingRobots();

    void setStatusFromFeedback();

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
    void pushControllerTransform(const transform::TransformWithVelocity& transform) {
        this->world_model->pushTransform(transform);
    }
    friend luhviz::DataProxy;

   public:
    const inline static std::string BALL_FRAME{"ball"};
    const inline static std::string BALL_PLACEMENT_FRAME{"ball_placement_position"};
    const inline static transform::Position BALL_POSITION{BALL_FRAME};
    const inline static transform::Position BALL_PLACEMENT_POSITION{BALL_PLACEMENT_FRAME};

   private:
    // TODO change per config or by values from game controller
    constexpr inline static size_t MAX_ACTIVE_ROBOTS_PER_TEAM = 6;
    constexpr inline static time::Duration MAX_ROBOT_MISSING_TIME = time::Duration(0.2);

    event_system::EventSystem& event_system;

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
    ssl_interface::SSLStage current_game_stage;
    TeamInfo ally_team_info{};
    TeamInfo enemy_team_info{};

    std::unordered_map<RobotIdentifier, std::unique_ptr<RobotDataFilter>> robot_data_filters;
    std::unordered_map<RobotIdentifier, time::TimePoint> last_feedback_times;
    time::TimePoint last_filter_param_update{};
    time::Duration filter_update_time{1.0};

    std::atomic_bool should_run{true};

    std::unique_ptr<DataProcessor> data_processor;
    std::unique_ptr<BallFilter> ball_filter;

    std::mutex robot_infos_mutex;
    std::unordered_map<RobotIdentifier, marker::RobotInfo> robot_infos;

    marker::LinePlot frequency_plot;
    std::vector<time::LoopStopwatch> vision_freq_stopwatch;

    // The ball position when a kickoff, free kick or penalty kick was issued
    std::optional<Eigen::Vector2d> special_kick_ball_position;
    std::optional<time::TimePoint> special_kick_timestamp;

    logger::Logger logger{"game_data_provider"};
};
}  // namespace luhsoccer::game_data_provider