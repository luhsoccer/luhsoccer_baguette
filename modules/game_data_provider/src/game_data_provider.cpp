#include "game_data_provider/game_data_provider.hpp"

#include "data_processor.hpp"
#include "ball_filter.hpp"

#include "observer/continuous_observer.hpp"
#include "marker_service/marker_service.hpp"
#include "robot_interface/events.hpp"
#include "robot_interface/robot_interface_types.hpp"
#include "config_provider/config_store_main.hpp"
#include "config/game_config.hpp"
#include "config/game_data_provider_config.hpp"
#include "robot_data_filter.hpp"
#include "transform_helper/world_model_helper.hpp"
#include "core/visit.hpp"
#include "event_system/event_system.hpp"
#include "vision_processor/vision_processor_events.hpp"

#include <optional>
#include <chrono>
#include <utility>

#include <fmt/chrono.h>

namespace luhsoccer::game_data_provider {

GameDataProvider::GameDataProvider(marker::MarkerService& ms, event_system::EventSystem& event_system)
    : event_system(event_system),
      ms(ms),
      world_model(std::make_shared<transform::WorldModel>("world")),
      observer(std::make_shared<observer::Observer>(world_model, event_system)),
      current_game_stage(ssl_interface::SSLStage::NORMAL_FIRST_HALF_PRE),
      data_processor(std::make_unique<DataProcessor>()),
      ball_filter(std::make_unique<BallFilter>(world_model, ms)),
      frequency_plot("Frequencies") {
    for (const auto& robot : this->world_model->getPossibleRobots<Team::ALLY>()) {
        this->robot_data_filters.try_emplace(
            robot, std::make_unique<RobotDataFilter>(robot, this->world_model->getGlobalFrame()));

        frequency_plot.getLine(fmt::format("Robot {}", robot.id));
    }

    this->world_model->pushNewGameState(GameState::NORMAL, time::now());
    this->world_model->setGoalieId(this->goalie);
}

GameDataProvider::~GameDataProvider() = default;

void GameDataProvider::setup(event_system::EventSystem& system) {
    system.registerEventHandler<RealWorldModelUpdatedEvent>(
        [this](event_system::EventContext<RealWorldModelUpdatedEvent> /*ctx*/) { this->updateObserver(); });
    system.registerEventHandler<robot_interface::RobotCommandSendEvent>(
        [this](event_system::EventContext<robot_interface::RobotCommandSendEvent> ctx) {
            this->data_processor->appendData(ctx.event);
        },
        true);
    system.registerEventHandler<robot_interface::RobotFeedbackReceivedEvent>(
        [this](event_system::EventContext<robot_interface::RobotFeedbackReceivedEvent> ctx) {
            this->data_processor->appendData(ctx.event);
        },
        true);
    system.registerEventHandler<ssl_interface::NewFieldDataEvent>(
        [this](event_system::EventContext<ssl_interface::NewFieldDataEvent> ctx) {
            this->data_processor->appendData(ctx.event.data);
        },
        true);
    system.registerEventHandler<vision_processor::NewProcessedVisionDataEvent>(
        [this](event_system::EventContext<vision_processor::NewProcessedVisionDataEvent> ctx) {
            this->data_processor->appendData(ctx.event.data);
        },
        true);

    system.registerEventHandler<ssl_interface::NewGameControllerDataEvent>(
        [this](event_system::EventContext<ssl_interface::NewGameControllerDataEvent> ctx) {
            this->data_processor->appendData(ctx.event.data);
        },
        true);

    system.registerEventHandler<event_system::TimerEvent100Hz>(
        [this](event_system::EventContext<event_system::TimerEvent100Hz>) {
            this->removeMissingRobots();
            this->setStatusFromFeedback();
        });
    ball_filter->setup(system);
}

void GameDataProvider::setup() {
    // give marker service the real world model reference
    ms.setRealWorldmodel(this->world_model);

    size_t marker_id = 0;
    for (const RobotIdentifier& robot : this->world_model->getPossibleRobots()) {
        marker::Robot r{{robot.getFrame(), 0, 0, 0}, robot, "robots", marker_id++};
        r.setColor(robot.isAlly() ? marker::Color::BLUE() : marker::Color::YELLOW());
        ms.displayMarker(r);
        // display robot id above
        constexpr double TEXT_HEIGHT = 0.3;
        marker::Text id_text{{robot.getFrame(), -0.2, 0, 0}, "robots", marker_id++};
        id_text.setText(std::to_string(robot.id));
        id_text.setHeight(TEXT_HEIGHT);
        id_text.setColor(robot.isAlly() ? marker::Color::WHITE() : marker::Color::BLACK());
        id_text.setScale({3});
        ms.displayMarker(id_text);
    }
}

void GameDataProvider::stop() { this->should_run = false; }

void GameDataProvider::loop(std::atomic_bool& /*should_run*/) {
    this->data_processor->waitForNewData([this](auto&& data) {
        auto visitor = overload{
            [this](const ssl_interface::SSLFieldData& field_data) { this->onNewFieldData(field_data); },
            [this](const ssl_interface::SSLGameControllerData& gc_data) { this->onNewGameControllerData(gc_data); },
            [this](const robot_interface::RobotFeedbackReceivedEvent& robot_feedback) {
                this->onNewRobotFeedback(robot_feedback);
            },
            [this](const robot_interface::RobotCommandSendEvent& robot_command) {
                this->onNewRobotCommand(robot_command);
            },

            [this](const vision_processor::ProcessedVisionData& processed_vision_data) {
                this->onNewProcessedVisionData(processed_vision_data);
            }};

        std::visit(visitor, std::forward<decltype(data)>(data));
    });
}

void GameDataProvider::onNewProcessedVisionData(const vision_processor::ProcessedVisionData& data) {
    bool mirror = config_provider::ConfigProvider::getConfigStore().game_config.is_flipped;
    // Lock the data from other while we're replacing the old data
    const std::unique_lock lock(this->vision_data_mutex);
    if (data.camera_id >= this->vision_freq_stopwatch.size()) {
        for (size_t id = this->vision_freq_stopwatch.size(); id < data.camera_id + 1; id++) {
            this->vision_freq_stopwatch.emplace_back(fmt::format("Camera{}", id), 100, 100);
        }
    }
    this->vision_freq_stopwatch[data.camera_id].tik();
    /// @todo change to vision time, but we have to sync first
    this->vision_data_timestamp = time::now();  // data.timestamp_capture;
    {
        // this->ally_robots.clear();
        // this->enemy_robots.clear();
        // this->robots.clear();

        // constexpr size_t NUM_ROBOTS = 16;

        /// @todo refactor this code
        constexpr double RADIAN_TO_DEGREES = 180.0 / L_PI;
        for (size_t robot_id = 0; robot_id < data.blue_robots.size(); robot_id++) {
            if (!data.blue_robots[robot_id].has_value()) {
                continue;
            }
            auto transform = *data.blue_robots[robot_id];

            if (mirror) {
                transform.translation().x() *= -1.0;
                transform.translation().y() *= -1.0;
                transform.rotate(Eigen::Rotation2Dd(L_PI));
            }

            const auto handle = createRIDFromId<TeamColor::BLUE>(robot_id);
            if (handle.isAlly()) {
                // Info marker

                std::lock_guard lock(this->robot_infos_mutex);
                auto info_it = this->robot_infos.find(handle);
                if (info_it == this->robot_infos.end()) {
                    marker::RobotInfo info(handle);
                    info_it = this->robot_infos.insert_or_assign(handle, info).first;
                    info_it->second.addBadge("1-Status",
                                             {"Disconnected", marker::Color::BLACK(), marker::Color::WHITE()});
                }
                marker::RobotInfo& robot_info = info_it->second;
                // robot_info.addBadge("1-Status", {"Connected", marker::Color::GREEN(), marker::Color::WHITE()});
                // robot_info.addParam(
                //     "Vision Coordinates",
                //     fmt::format("({:0.3f}, {:0.3f}, {:0.3f})", transform.translation().x(),
                //     transform.translation().y(),
                //                 Eigen::Rotation2Dd(transform.rotation()).angle() * RADIAN_TO_DEGREES));
                auto filter_it = this->robot_data_filters.find(handle);
                if (filter_it != this->robot_data_filters.end()) {
                    if (filter_it->second->addVisionData(
                            transform,
                            config_provider::ConfigProvider::getConfigStore().game_data_provider_config.vision_delay_ms,
                            this->vision_data_timestamp)) {
                        this->world_model->pushTransform(filter_it->second->getLatestTransform());
                        this->world_model->pushAllyRobotData(handle, filter_it->second->getLatestRobotData());

                    } else if (config_provider::ConfigProvider::getConfigStore()
                                   .game_data_provider_config.show_vision_robot_position) {
                        publishRobotToWorldModel(transform, handle, "_vision");
                    }
                    robot_info.addParam("", fmt::format("{}", filter_it->second->getFilterMode()));
                    this->ms.displayMarker(robot_info);
                }
            } else {
                publishRobotToWorldModel(transform, handle);
                publishRobotData(handle);
            }
        }

        for (size_t robot_id = 0; robot_id < data.yellow_robots.size(); robot_id++) {
            if (!data.yellow_robots[robot_id].has_value()) {
                continue;
            }
            auto transform = *data.yellow_robots[robot_id];

            if (mirror) {
                transform.translation().x() *= -1.0;
                transform.translation().y() *= -1.0;
                transform.rotate(Eigen::Rotation2Dd(L_PI));
            }
            const auto handle = createRIDFromId<TeamColor::YELLOW>(robot_id);
            if (handle.isAlly()) {
                // Info marker

                std::lock_guard lock(this->robot_infos_mutex);
                auto info_it = this->robot_infos.find(handle);
                if (info_it == this->robot_infos.end()) {
                    marker::RobotInfo info(handle);
                    info_it = this->robot_infos.insert_or_assign(handle, info).first;
                    info_it->second.addBadge("1-Status",
                                             {"Disconnected", marker::Color::BLACK(), marker::Color::WHITE()});
                }
                marker::RobotInfo& robot_info = info_it->second;
                // robot_info.addBadge("1-Status", {"Connected", marker::Color::GREEN(), marker::Color::WHITE()});
                // robot_info.addParam(
                //     "Vision Coordinates",
                //     fmt::format("({:0.3f}, {:0.3f}, {:0.3f})", transform.translation().x(),
                //     transform.translation().y(),
                //                 Eigen::Rotation2Dd(transform.rotation()).angle() * RADIAN_TO_DEGREES));
                auto filter_it = this->robot_data_filters.find(handle);
                if (filter_it != this->robot_data_filters.end()) {
                    if (filter_it->second->addVisionData(
                            transform,
                            config_provider::ConfigProvider::getConfigStore().game_data_provider_config.vision_delay_ms,
                            this->vision_data_timestamp)) {
                        this->world_model->pushTransform(filter_it->second->getLatestTransform());
                        this->world_model->pushAllyRobotData(handle, filter_it->second->getLatestRobotData());

                    } else if (config_provider::ConfigProvider::getConfigStore()
                                   .game_data_provider_config.show_vision_robot_position) {
                        publishRobotToWorldModel(transform, handle, "_vision");
                    }
                    robot_info.addParam("", fmt::format("{}", filter_it->second->getFilterMode()));
                    this->ms.displayMarker(robot_info);
                }
            } else {
                publishRobotToWorldModel(transform, handle);
                publishRobotData(handle);
            }
        }
        marker::Info vision_info("Vision", 0);
        std::string msg = fmt::format("{:0.0f}Hz", this->vision_freq_stopwatch[data.camera_id].measuredFrequency());
        vision_info.set(fmt::format("Frequency Camera{}", data.camera_id), msg);
        this->ms.displayMarker(vision_info);

        // removeMissingRobots();
    }

    {
        static bool is_blue_old = config_provider::ConfigProvider::getConfigStore().game_config.is_blue;
        bool is_blue_curr = config_provider::ConfigProvider::getConfigStore().game_config.is_blue;

        if (is_blue_curr != is_blue_old) {
            this->logger.info("Flipped Sides. Removing old robots from field...!");

            for (const auto& robot : this->world_model->getVisibleRobots()) {
                event_system.fireEvent(RobotRemovedFromFieldEvent(transform::RobotHandle(robot, this->world_model)));
                this->world_model->removeRobotFromField(robot);
            }
            is_blue_old = is_blue_curr;
        }

        std::unique_lock color_lock{this->team_color_mutex};

        if (is_blue_curr) {
            this->our_team_color = TeamColor::BLUE;
        } else {
            this->our_team_color = TeamColor::YELLOW;
        }
    }

    if (time::now() - this->last_filter_param_update > this->filter_update_time) {
        for (auto& filter : this->robot_data_filters) {
            filter.second->updateParams();
        }
        this->last_filter_param_update = time::now();
    }

    this->event_system.fireEvent(RealWorldModelUpdatedEvent(this->world_model, data.time_captured, data.time_sent));
}

TeamInfo GameDataProvider::processTeamInfo(const ssl_interface::SSLTeamInfo& info) {
    /// @todo change this
    TeamInfo team_info{info.name,
                       info.score,
                       info.red_cards,
                       info.yellow_cards,
                       info.yellow_card_times,
                       info.timeouts,
                       info.timeout_time,
                       info.goalkeeper,
                       info.foul_counter,
                       info.ball_placement_failures,
                       info.can_place_ball,
                       info.max_allowed_bots,
                       info.bot_substitution_intent,
                       info.ball_placement_failures_reached};

    return team_info;
}

void GameDataProvider::onNewGameControllerData(const ssl_interface::SSLGameControllerData& data) {
    bool mirror = config_provider::ConfigProvider::getConfigStore().game_config.is_flipped;
    const std::unique_lock lock(this->gamecontroller_data_mutex);
    if (this->gamecontroller_data_timestamp != data.timestamp_issued) {
        this->gamecontroller_data_timestamp = data.timestamp_issued;
        GameState old_state = this->current_state;
        this->current_state = nextGameState(data);

        if (data.stage == ssl_interface::SSLStage::POST_GAME) {
            // So that the robots can dance
            this->current_state = GameState::STOP;
        }

        if (this->current_game_stage != data.stage) {
            this->current_game_stage = data.stage;
            this->event_system.fireEvent(GameStageChangedEvent(this->current_game_stage));
        }

        if (old_state != this->current_state) {
            event_system.fireEvent(GameStateChangedEvent(old_state, this->current_state));
        }
        this->world_model->pushNewGameState(current_state, time::now());
        logger.info("Got new game state {}", this->current_state);
    }
    if (this->our_team_color == TeamColor::BLUE) {
        this->ally_team_info = processTeamInfo(data.blue_team_info);
        this->enemy_team_info = processTeamInfo(data.yellow_team_info);
    } else {
        this->ally_team_info = processTeamInfo(data.yellow_team_info);
        this->enemy_team_info = processTeamInfo(data.blue_team_info);
    }

    if ((this->current_state == GameState::BALL_PLACEMENT_ENEMY ||
         this->current_state == GameState::BALL_PLACEMENT_FORCE_START ||
         this->current_state == GameState::BALL_PLACEMENT_FREE_KICK) &&
        data.designated_position) {
        transform::TransformWithVelocity v;
        v.header.stamp = time::now();  // TODO use ssl time
        v.header.parent_frame = world_model->getGlobalFrame();
        v.header.child_frame = BALL_PLACEMENT_FRAME;

        if (mirror) {
            v.transform =
                Eigen::Translation2d(data.designated_position->x() * -1.0, data.designated_position->y() * -1.0);

        } else {
            v.transform = Eigen::Translation2d(data.designated_position->x(), data.designated_position->y());
        }

        world_model->pushTransform(std::move(v), true);
    }

    this->goalie = RobotIdentifier(this->ally_team_info.goalkeeper, Team::ALLY);
    this->world_model->setGoalieId(this->goalie);
    this->enemy_goalie = RobotIdentifier(this->enemy_team_info.goalkeeper, Team::ENEMY);

    this->updateGameState();
}

void GameDataProvider::onNewFieldData(const ssl_interface::SSLFieldData& data) {
    // create center and goal positions

    transform::Position center{this->world_model->getGlobalFrame()};
    transform::Position goal_left{
        this->world_model->getGlobalFrame(),
        data.penalty_area_left_field_bottom.x() +
            (data.penalty_area_left_baseline_bottom.x() - data.penalty_area_left_field_bottom.x()) / 2,
        (data.penalty_area_left_baseline_top.y() + data.penalty_area_left_baseline_bottom.y()) / 2, 0};

    transform::Position goal_right{
        this->world_model->getGlobalFrame(),
        data.penalty_area_right_field_bottom.x() +
            (data.penalty_area_right_baseline_bottom.x() - data.penalty_area_right_field_bottom.x()) / 2,
        (data.penalty_area_right_baseline_top.y() + data.penalty_area_right_baseline_bottom.y()) / 2, 0};

    // parameters
    const double line_thickness = 0.015;
    size_t id = 0;
    const double ground_scale_factor = 1.2;
    const double default_ground_height = -0.001;
    const marker::Color default_ground_color = marker::Color::hsv2Rgb(120, 100, 60, 1);

    // display green field ground
    marker::Rect field_ground(center, "ground", 1);
    field_ground.setSize({data.size.x() * ground_scale_factor, data.size.y() * ground_scale_factor});
    field_ground.setFilled(true);
    field_ground.setHeight(default_ground_height);
    field_ground.setColor(default_ground_color);
    ms.displayMarker(field_ground);

    // outer border lines
    marker::Rect field_border{center, "SSLFieldLines", id++};
    field_border.setSize({data.field_right_bottom.x() - data.field_left_bottom.x(),
                          data.field_left_top.y() - data.field_left_bottom.y()});
    field_border.setThickness(line_thickness);
    field_border.setColor(marker::Color::WHITE());
    ms.displayMarker(field_border);

    // horizontal middle line
    marker::Line center_line_horizontal{center, "SSLFieldLines", id++};
    center_line_horizontal.setLinePoints({data.field_right_center.x(), data.field_right_center.y()},
                                         {data.field_left_center.x(), data.field_left_center.y()});
    center_line_horizontal.setThickness(line_thickness);
    center_line_horizontal.setColor(marker::Color::WHITE());
    ms.displayMarker(center_line_horizontal);

    // vertical middle line
    marker::Line center_line_vertical{center, "SSLFieldLines", id++};
    center_line_vertical.setLinePoints({data.field_top_center.x(), data.field_top_center.y()},
                                       {data.field_bottom_center.x(), data.field_bottom_center.y()});
    center_line_vertical.setThickness(line_thickness);
    center_line_vertical.setColor(marker::Color::WHITE());
    ms.displayMarker(center_line_vertical);

    // left goal area lines
    marker::Rect goal_area_left{goal_left, "SSLFieldLines", id++};
    goal_area_left.setSize({data.penalty_area_left_field_bottom.x() - data.penalty_area_left_baseline_bottom.x(),
                            data.penalty_area_left_field_top.y() - data.penalty_area_left_field_bottom.y()});
    goal_area_left.setThickness(line_thickness);
    goal_area_left.setColor(marker::Color::WHITE());
    ms.displayMarker(goal_area_left);

    // right goal area lines
    marker::Rect goal_area_right{goal_right, "SSLFieldLines", id++};
    goal_area_right.setSize({data.penalty_area_right_field_bottom.x() - data.penalty_area_right_baseline_bottom.x(),
                             data.penalty_area_right_field_top.y() - data.penalty_area_right_field_bottom.y()});
    goal_area_right.setThickness(line_thickness);
    goal_area_right.setColor(marker::Color::WHITE());
    ms.displayMarker(goal_area_right);

    // field arc lines
    for (const ssl_interface::SSLFieldArc& arc : data.arcs) {
        marker::Circle c{center, "SSLFieldLines", id};
        c.setRadius(arc.radius);
        c.setThickness(line_thickness);
        c.setColor(marker::Color::WHITE());
        ms.displayMarker(c);
        id++;
    }

    // goal border models
    marker::GoalBorderDivA goal_border_right{
        {this->world_model->getGlobalFrame(), data.field_right_center.x(), data.field_right_center.y(), L_PI / 2},
        "SSLFieldLines",
        id++};
    goal_border_right.setColor(marker::Color::GREY());
    marker::GoalBorderDivA goal_border_left{
        {this->world_model->getGlobalFrame(), data.field_left_center.x(), data.field_left_center.y(), 3 * L_PI / 2},
        "SSLFieldLines",
        id++};
    goal_border_left.setColor(marker::Color::GREY());
    ms.displayMarker(goal_border_right);
    ms.displayMarker(goal_border_left);

    // publish frames
    auto publish_static_frame = [this](std::string name, const Eigen::Vector2d& position) {
        transform::TransformWithVelocity trans_with_vel;
        trans_with_vel.transform = Eigen::Translation2d(position) * Eigen::Rotation2Dd(0.0);
        trans_with_vel.header.child_frame = std::move(name);
        trans_with_vel.header.stamp = time::now();
        this->world_model->pushTransform(trans_with_vel, true);
    };
    publish_static_frame(transform::field::CENTER, data.field_center);
    publish_static_frame(transform::field::MID_LINE_LEFT, data.field_top_center);
    publish_static_frame(transform::field::MID_LINE_RIGHT, data.field_bottom_center);

    publish_static_frame(transform::field::CORNER_ENEMY_LEFT, data.field_right_top);
    publish_static_frame(transform::field::CORNER_ENEMY_RIGHT, data.field_right_bottom);
    publish_static_frame(transform::field::CORNER_ALLY_LEFT, data.field_left_top);
    publish_static_frame(transform::field::CORNER_ALLY_RIGHT, data.field_left_bottom);

    publish_static_frame(transform::field::GOAL_ENEMY_LEFT, data.goal_right_top);
    publish_static_frame(transform::field::GOAL_ENEMY_RIGHT, data.goal_right_bottom);
    publish_static_frame(transform::field::GOAL_ENEMY_CENTER, data.field_right_center);

    publish_static_frame(transform::field::GOAL_ALLY_LEFT, data.goal_left_top);
    publish_static_frame(transform::field::GOAL_ALLY_RIGHT, data.goal_left_bottom);
    publish_static_frame(transform::field::GOAL_ALLY_CENTER, data.field_left_center);

    publish_static_frame(transform::field::DEFENSE_AREA_CORNER_ENEMY_LEFT, data.penalty_area_right_field_top);
    publish_static_frame(transform::field::DEFENSE_AREA_CORNER_ENEMY_RIGHT, data.penalty_area_right_field_bottom);
    publish_static_frame(transform::field::DEFENSE_AREA_CORNER_ALLY_LEFT, data.penalty_area_left_field_top);
    publish_static_frame(transform::field::DEFENSE_AREA_CORNER_ALLY_RIGHT, data.penalty_area_left_field_bottom);
    publish_static_frame(transform::field::DEFENSE_AREA_INTERSECTION_ENEMY_LEFT, data.penalty_area_right_baseline_top);
    publish_static_frame(transform::field::DEFENSE_AREA_INTERSECTION_ENEMY_RIGHT,
                         data.penalty_area_right_baseline_bottom);
    publish_static_frame(transform::field::DEFENSE_AREA_INTERSECTION_ALLY_LEFT, data.penalty_area_left_baseline_top);
    publish_static_frame(transform::field::DEFENSE_AREA_INTERSECTION_ALLY_RIGHT,
                         data.penalty_area_left_baseline_bottom);

    transform::FieldData field_data;
    field_data.division = data.division;
    field_data.size = data.size;
    field_data.goal_width = data.goal_width;
    field_data.goal_depth = data.goal_depth;
    field_data.boundary_width = data.boundary_width;
    field_data.penalty_area_depth = data.penalty_area_depth;
    field_data.penalty_area_width = data.penalty_area_width;
    field_data.center_circle_radius = data.center_circle_radius;
    field_data.line_thickness = data.line_thickness;
    field_data.goal_center_to_penalty_mark = data.goal_center_to_penalty_mark;
    field_data.goal_height = data.goal_height;
    field_data.ball_radius = data.ball_radius;
    field_data.max_robot_radius = data.max_robot_radius;
    field_data.field_runoff_width = data.boundary_width;

    if (this->world_model->getFieldData() != field_data) {
        this->world_model->setFieldData(field_data);
        this->event_system.fireEvent(FieldDataUpdatedEvent(field_data));
    }
}

RobotIdentifier GameDataProvider::getGoalie() const {
    std::lock_guard lock(this->gamecontroller_data_mutex);
    return this->goalie;
}

RobotIdentifier GameDataProvider::getEnemyGoalie() const {
    std::lock_guard lock(this->gamecontroller_data_mutex);
    return this->enemy_goalie;
}

void GameDataProvider::onNewRobotCommand(const robot_interface::RobotCommandSendEvent& data) {
    robot_interface::RobotCommand command = data.command;
    command.time_sent = time::now();
    auto filter_it = this->robot_data_filters.find(data.id);
    if (filter_it != this->robot_data_filters.end() && filter_it->second->addCommandSend(command)) {
        auto trans = filter_it->second->getLatestTransform();
        // trans.header.child_frame += "_vision";
        this->world_model->pushTransform(trans);
        this->world_model->pushAllyRobotData(data.id, filter_it->second->getLatestRobotData());
    }

    if (data.command.kick_command.has_value()) {
        std::lock_guard lock(this->robot_infos_mutex);
        auto info_it = this->robot_infos.find(data.id);
        if (info_it != this->robot_infos.end()) {
            bool chip = data.command.kick_command->type == robot_interface::KickType::CHIP;
            info_it->second.addBadge("9-Chip", {chip ? "Chip" : "", marker::Color::RED(), marker::Color::WHITE()});
        }
    }
}

void GameDataProvider::onNewRobotFeedback(const robot_interface::RobotFeedbackReceivedEvent& feedback) {
    bool is_mirrored = config_provider::ConfigProvider::getConfigStore().game_config.is_flipped.val();

    const RobotIdentifier& id = feedback.id;
    robot_interface::RobotFeedback data = feedback.feedback;

    this->last_feedback_times[id] = time::now();

    if (is_mirrored && data.position) {
        data.position->x() *= -1.0;
        data.position->y() *= -1.0;
        data.position->z() += L_PI;
    }

    /// @todo sync robot time with our time
    data.time_stamp = time::now();

    if (config_provider::ConfigProvider::getConfigStore().game_data_provider_config.show_true_robot_position &&
        data.position.has_value()) {
        Eigen::Affine2d affine = Eigen::Translation2d(data.position->head(2)) * Eigen::Rotation2Dd(data.position->z());

        this->publishRobotToWorldModel(affine, id, "_true");
        data.position = std::nullopt;
    }

    auto filter_it = this->robot_data_filters.find(id);
    if (config_provider::ConfigProvider::getConfigStore().game_data_provider_config.use_robot_feedback_in_filter &&
        filter_it != this->robot_data_filters.end() && filter_it->second->addFeedbackData(data)) {
        this->world_model->pushTransform(filter_it->second->getLatestTransform());
        this->world_model->pushAllyRobotData(id, filter_it->second->getLatestRobotData());
    }

    // publish ball in dribbler marker
    if (data.has_ball.has_value() && data.has_ball.value()) {
        // NOLINTBEGIN
        marker::Circle has_ball({id.getFrame(), 0.1, -0.1}, "hasBall", id.id);
        has_ball.setRadius(0.02);
        has_ball.setFilled(true);
        has_ball.setColor(marker::Color::ORANGE());
        has_ball.setHeight(0.3);
        has_ball.setLifetime(0.1);
        this->ms.displayMarker(has_ball);
        /// NOLINTEND
    }

    std::lock_guard lock(this->robot_infos_mutex);
    auto info_it = this->robot_infos.find(id);

    if (info_it == this->robot_infos.end()) {
        marker::RobotInfo info(feedback.id);
        this->robot_infos.emplace(id, info);
        info_it = this->robot_infos.find(id);
    }

    info_it->second.addBadge(
        "8-Has Ball", {data.has_ball.value_or(false) ? "Ball" : "", marker::Color::ORANGE(), marker::Color::WHITE()});

    // if (info_it != this->robot_infos.end() && data.velocity) {
    //     info_it->second.addParam("Velocity", fmt::format("X: {: .2f}, Y: {: .2f}, Z: {: .2f}", data.velocity->x(),
    //                                                      data.velocity->y(), data.velocity->z()));
    // }

    if (info_it != this->robot_infos.end() && data.telemetry.has_value()) {
        info_it->second.addBadge("2-Battery Voltage", {fmt::format("{:2.2f}V", data.telemetry->battery_voltage),
                                                       marker::Color::BLUE(), marker::Color::WHITE()});
        // info_it->second.addParam("Cap Voltage", data.telemetry->capacitor_voltage);

        if (data.telemetry_rf) {
            // info_it->second.addParam("RSSI (Robot)", data.telemetry_rf->rssi_robot);
            // info_it->second.addParam("RSSI (Basestation)", data.telemetry_rf->rssi_base_station);
            info_it->second.addBadge("3-Frequency", {fmt::format("{:2.0f}Hz", data.telemetry_rf->frequency),
                                                     marker::Color::YELLOW(), marker::Color::BLACK()});

            this->frequency_plot.addPoint(feedback.id.id, static_cast<float>(time::now().asSec()),
                                          static_cast<float>(data.telemetry_rf->frequency));
            this->ms.displayMarker(this->frequency_plot);

            marker::Color color = marker::Color::GREEN();
            if (data.telemetry_rf->frequency < 40.0) {
                color = marker::Color::RED();
            } else if (data.telemetry_rf->frequency < 60.0) {
                color = marker::Color::ORANGE();
            } else if (data.telemetry_rf->frequency < 80.0) {
                color = marker::Color::YELLOW();
            }
            info_it->second.addBadge("1-Status", {"Connected", color, marker::Color::BLACK()});
        } else {
            info_it->second.addBadge("1-Status", {"Connected", marker::Color::GREEN(), marker::Color::BLACK()});
        }
    } else {
        info_it->second.addBadge("1-Status", {"Connected", marker::Color::GREEN(), marker::Color::BLACK()});
    }
    this->ms.displayMarker(info_it->second);
}
void GameDataProvider::publishRobotToWorldModel(const ssl_interface::SSLRobotInfo& info, const RobotIdentifier& id,
                                                const std::string& appendix) {
    this->publishRobotToWorldModel(info.transform, id, appendix);
}
void GameDataProvider::publishRobotToWorldModel(const Eigen::Affine2d& affine, const RobotIdentifier& handle,
                                                const std::string& appendix) {
    transform::TransformWithVelocity v;
    v.header.stamp = appendix.empty() ? this->vision_data_timestamp : time::now();
    v.header.parent_frame = world_model->getGlobalFrame();
    v.header.child_frame = handle.getFrame();
    v.header.child_frame += appendix;
    v.transform = affine;
    world_model->pushTransform(std::move(v));
}

/// @todo this has to be changed to add robot data like ball in dribbler etc.
void GameDataProvider::publishRobotData(const RobotIdentifier& id) {
    if (id.isAlly()) {
        transform::AllyRobotData d;
        d.on_field = true;
        d.time = this->vision_data_timestamp;
        this->world_model->pushAllyRobotData(id, d);
    } else {
        transform::RobotData d;
        d.on_field = true;
        d.time = this->vision_data_timestamp;
        this->world_model->pushEnemyRobotData(id, d);
    }
}

void GameDataProvider::removeMissingRobots() {
    for (auto& robot : this->world_model->getVisibleRobots()) {
        auto last_robot_data = this->world_model->getRobotData(robot);
        if (!last_robot_data || !last_robot_data->on_field ||
            (time::now() - last_robot_data->time) > MAX_ROBOT_MISSING_TIME) {
            event_system.fireEvent(RobotRemovedFromFieldEvent(transform::RobotHandle(robot, this->world_model)));
            this->world_model->removeRobotFromField(robot);
            // this->ms.removeRobotInfoMarker(robot);
        }
    }
}

void GameDataProvider::setStatusFromFeedback() {
    for (auto& [robot, last_feedback_time] : this->last_feedback_times) {
        if (time::now() - last_feedback_time > MAX_ROBOT_MISSING_TIME) {
            auto robot_data = this->world_model->getRobotData(robot);
            if (!robot_data.has_value() || !robot_data->on_field) {
                // this->ms.removeRobotInfoMarker(robot);
            } else {
                std::lock_guard lock(this->robot_infos_mutex);
                auto info_it = this->robot_infos.find(robot);
                if (info_it != this->robot_infos.end()) {
                    info_it->second.addBadge("1-Status",
                                             {"Disconnected", marker::Color::BLACK(), marker::Color::WHITE()});
                    this->ms.displayMarker(info_it->second);
                }
            }
        }
    }
}

[[nodiscard]] ssl_interface::SSLStage GameDataProvider::getGameStage() const { return this->current_game_stage; }

void GameDataProvider::updateGameState() {
    const std::lock_guard lock(this->game_state_mutex);

    const double ball_radius =
        config_provider::ConfigProvider::getConfigStore().game_data_provider_config.state_change_distance.val();

    const double state_time =
        config_provider::ConfigProvider::getConfigStore().game_data_provider_config.state_change_time.val();

    switch (this->current_state) {
        case transform::GameState::KICKOFF:
        case transform::GameState::KICKOFF_ENEMY:
        case transform::GameState::FREE_KICK:
        case transform::GameState::FREE_KICK_ENEMY: {
            bool switch_to_normal = false;
            auto current_ball_pos = transform::helper::getBallPosition(*world_model);
            if (current_ball_pos && special_kick_ball_position) {
                auto diff = special_kick_ball_position.value() - current_ball_pos.value();
                if (diff.norm() > ball_radius) {
                    logger.info("Switching to normal game state because ball moved more than {} m", ball_radius);
                    switch_to_normal = true;
                }
            }
            if (special_kick_timestamp && (static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(
                                                                   time::now() - special_kick_timestamp.value())
                                                                   .count()) > state_time)) {
                logger.info("Switching to normal game state because no state change after {} seconds", state_time);
                switch_to_normal = true;
            }

            if (switch_to_normal) {
                this->current_state = transform::GameState::NORMAL;
                this->world_model->pushNewGameState(this->current_state, time::now());
                this->special_kick_ball_position = std::nullopt;
                this->special_kick_timestamp = std::nullopt;
            }

            break;
        }
        case transform::GameState::PENALTY_PREP_ENEMY: {
            bool switch_to_penalty = false;
            auto current_ball_pos = transform::helper::getBallPosition(*world_model);
            if (current_ball_pos && special_kick_ball_position) {
                auto diff = special_kick_ball_position.value() - current_ball_pos.value();
                if (diff.norm() > ball_radius) {
                    logger.info("Switching to penalty enemy game state because ball moved more than {} m", ball_radius);
                    switch_to_penalty = true;
                }
            }
            if (special_kick_timestamp && (static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(
                                                                   time::now() - special_kick_timestamp.value())
                                                                   .count()) > state_time)) {
                logger.info("Switching to penalty enemy game state because no state change after {} seconds",
                            state_time);
                switch_to_penalty = true;
            }

            if (switch_to_penalty) {
                this->current_state = transform::GameState::PENALTY_ENEMY;
                this->world_model->pushNewGameState(this->current_state, time::now());
                this->special_kick_ball_position = std::nullopt;
                this->special_kick_timestamp = std::nullopt;
            }

            break;
        }
        default:
            break;
    }
}

GameState GameDataProvider::nextGameState(const ssl_interface::SSLGameControllerData& data) {
    // data.print();
    const std::lock_guard lock(this->game_state_mutex);
    const auto type = data.command;
    using namespace ssl_interface;
    switch (type) {
        case SSLCommandType::HALT:
            return GameState::HALT;
        case SSLCommandType::STOP:
            return GameState::STOP;
        case SSLCommandType::NORMAL_START:
            special_kick_ball_position = transform::helper::getBallPosition(*world_model);
            special_kick_timestamp = time::now();
            switch (this->current_state) {
                case GameState::KICKOFF_PREP:
                    return GameState::KICKOFF;
                case GameState::KICKOFF_PREP_ENEMY:
                    return GameState::KICKOFF_ENEMY;
                case GameState::FREE_KICK_PREP:
                    return GameState::FREE_KICK;
                case GameState::FREE_KICK_PREP_ENEMY:
                    return GameState::FREE_KICK_ENEMY;
                case GameState::PENALTY_PREP:
                    return GameState::PENALTY;
                case GameState::PENALTY_PREP_ENEMY:
                    return GameState::PENALTY_PREP_ENEMY;
                    // return GameState::PENALTY_ENEMY;
                default:
                    return GameState::NORMAL;
            }
        case SSLCommandType::FORCE_START:
            return GameState::NORMAL;
        case SSLCommandType::PREPARE_KICKOFF_BLUE:
            if (this->our_team_color == TeamColor::BLUE) {
                return GameState::KICKOFF_PREP;
            } else {
                return GameState::KICKOFF_PREP_ENEMY;
            }
        case SSLCommandType::PREPARE_KICKOFF_YELLOW:
            if (this->our_team_color == TeamColor::YELLOW) {
                return GameState::KICKOFF_PREP;
            } else {
                return GameState::KICKOFF_PREP_ENEMY;
            }
        case SSLCommandType::PREPARE_PENALTY_BLUE:
            if (this->our_team_color == TeamColor::BLUE) {
                return GameState::PENALTY_PREP;
            } else {
                return GameState::PENALTY_PREP_ENEMY;
            }
        case SSLCommandType::PREPARE_PENALTY_YELLOW:
            if (this->our_team_color == TeamColor::YELLOW) {
                return GameState::PENALTY_PREP;
            } else {
                return GameState::PENALTY_PREP_ENEMY;
            }
        case SSLCommandType::DIRECT_FREE_BLUE:
            special_kick_ball_position = transform::helper::getBallPosition(*world_model);
            special_kick_timestamp = time::now();
            if (this->our_team_color == TeamColor::BLUE) {
                return GameState::FREE_KICK;
            } else {
                return GameState::FREE_KICK_ENEMY;
            }
        case SSLCommandType::DIRECT_FREE_YELLOW:
            special_kick_ball_position = transform::helper::getBallPosition(*world_model);
            special_kick_timestamp = time::now();
            if (this->our_team_color == TeamColor::YELLOW) {
                return GameState::FREE_KICK;
            } else {
                return GameState::FREE_KICK_ENEMY;
            }
        case SSLCommandType::TIMEOUT_BLUE:

            if (this->our_team_color == TeamColor::BLUE) {
                return GameState::TIMEOUT_ALLY;
            } else {
                return GameState::TIMEOUT_ENEMY;
            }

        case SSLCommandType::TIMEOUT_YELLOW:

            if (this->our_team_color == TeamColor::YELLOW) {
                return GameState::TIMEOUT_ALLY;
            } else {
                return GameState::TIMEOUT_ENEMY;
            }

        case SSLCommandType::BALL_PLACEMENT_BLUE:
            if (this->our_team_color == TeamColor::BLUE) {
                if (data.next_command.has_value()) {
                    switch (*data.next_command) {
                        case ssl_interface::SSLCommandType::DIRECT_FREE_BLUE:
                        case ssl_interface::SSLCommandType::INDIRECT_FREE_BLUE:
                            // This needs to be changed to a new gamestate (& be implemented in the strtegy & planner)if
                            // we want to take advantage of doing an instant free kick after successfull ball placement
                            // https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_placement
                            return GameState::BALL_PLACEMENT_FORCE_START;

                        default:
                            return GameState::BALL_PLACEMENT_FORCE_START;
                    }
                } else {
                    return GameState::BALL_PLACEMENT_FORCE_START;
                }
            } else {
                return GameState::BALL_PLACEMENT_ENEMY;
            }
        case SSLCommandType::BALL_PLACEMENT_YELLOW:
            if (this->our_team_color == TeamColor::YELLOW) {
                if (data.next_command.has_value()) {
                    switch (*data.next_command) {
                        case ssl_interface::SSLCommandType::DIRECT_FREE_YELLOW:
                        case ssl_interface::SSLCommandType::INDIRECT_FREE_YELLOW:

                            // This needs to be changed to a new gamestate (& be implemented in the strtegy & planner)if
                            // we want to take advantage of doing an instant free kick after successfull ball placement
                            // https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_placement
                            return GameState::BALL_PLACEMENT_FORCE_START;
                        default:
                            return GameState::BALL_PLACEMENT_FORCE_START;
                    }
                } else {
                    return GameState::BALL_PLACEMENT_FORCE_START;
                }
            } else {
                if (data.designated_position) {
                    logger.info("Place placement {} {}", data.designated_position->x(), data.designated_position->y());
                } else {
                    logger.info("Got ball placement without position");
                }
                return GameState::BALL_PLACEMENT_ENEMY;
            }
        case SSLCommandType::GOAL_BLUE:
        case SSLCommandType::GOAL_YELLOW:
            return this->current_state;
        default:
            return GameState::HALT;
    }
}

time::TimePoint GameDataProvider::getVisionDataTimestamp() const {
    const std::shared_lock lock(this->gamecontroller_data_mutex);
    return this->gamecontroller_data_timestamp;
}

time::TimePoint GameDataProvider::getGameControllerDataTimestamp() const {
    const std::shared_lock lock(this->gamecontroller_data_mutex);
    return this->gamecontroller_data_timestamp;
}

TeamInfo GameDataProvider::getAllyTeamInfo() const {
    const std::shared_lock lock(this->gamecontroller_data_mutex);
    return this->ally_team_info;
}
TeamInfo GameDataProvider::getEnemyTeamInfo() const {
    const std::shared_lock lock(this->gamecontroller_data_mutex);
    return this->enemy_team_info;
}

std::optional<Eigen::Vector2d> GameDataProvider::getSpecialKickPosition() {
    std::lock_guard lock(this->game_state_mutex);

    return this->special_kick_ball_position;
}

std::optional<time::TimePoint> GameDataProvider::getSpecialKickTime() {
    std::lock_guard lock(this->game_state_mutex);

    return this->special_kick_timestamp;
}

void GameDataProvider::updateObserver() {
    // Update the Observer once
    observer->update(*this);
    // Set (or remove) the ball-obtained position in the WorldModel
    const auto ball_obtained_pos = observer->getBallObtainedPos();
    if (ball_obtained_pos.has_value()) {
        // Only set the ballObtainedPos if it was not set so the original time is preserved
        if (world_model->getLastBallObtainPosition() == std::nullopt) {
            // convert the vector to an affine2d
            const Eigen::Affine2d ball_obtained_pos_affine =
                Eigen::Translation2d(*ball_obtained_pos) * Eigen::Rotation2Dd(0);
            world_model->setLastBallObtainPosition(time::now(), ball_obtained_pos_affine);
        }
    } else {
        world_model->removeLastBallObtainPosition();
    }
}

}  // namespace luhsoccer::game_data_provider
