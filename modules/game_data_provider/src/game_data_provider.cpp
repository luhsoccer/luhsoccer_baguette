#include "game_data_provider/game_data_provider.hpp"

#include "data_processor.hpp"

#include "observer/continuous_observer.hpp"
#include "marker_service/marker_service.hpp"
#include "robot_interface/robot_interface.hpp"
#include "ssl_interface/ssl_interface.hpp"
#include "robot_interface/robot_interface_types.hpp"
#include "config_provider/config_store_main.hpp"
#include "robot_data_filter.hpp"
#include "ball_filter/ball_filter.hpp"
#include "transform_helper/world_model_helper.hpp"
#include "visit.hpp"

#include <optional>
#include <chrono>
#include <utility>
namespace luhsoccer::game_data_provider {

using Data =
    std::variant<ssl_interface::SSLFieldData, ssl_interface::SSLVisionData, ssl_interface::SSLGameControllerData,
                 std::pair<RobotIdentifier, robot_interface::RobotFeedback>,
                 std::pair<uint32_t, robot_interface::RobotCommand>>;

GameDataProvider::GameDataProvider(ssl_interface::SSLInterface& ssl_interface,
                                   robot_interface::RobotInterface& robot_interface, marker::MarkerService& ms)
    : ssl_interface(ssl_interface),
      robot_interface(robot_interface),
      feedback_callback(
          std::make_shared<std::function<void(const std::pair<RobotIdentifier, robot_interface::RobotFeedback>&)>>(
              [this](auto&& data) { this->data_processor->appendData(std::forward<decltype(data)>(data)); })),
      command_callback(std::make_shared<std::function<void(const std::pair<uint32_t, robot_interface::RobotCommand>&)>>(
          [this](auto&& data) { this->data_processor->appendData(std::forward<decltype(data)>(data)); })),
      ms(ms),
      world_model(std::make_shared<transform::WorldModel>("world")),
      observer(std::make_shared<observer::Observer>(world_model)),
      ball_filter(
          std::make_unique<BallFilter>(world_model->getBallFrame(), world_model->getGlobalFrame(), world_model)),
      data_processor(std::make_unique<DataProcessor>()),
      frequency_plot("Frequencies"),
      ball_velocity_plot("Ball Velocity") {
    for (const auto& robot : this->world_model->getPossibleRobots<Team::ALLY>()) {
        this->robot_data_filters.try_emplace(
            robot, std::make_unique<RobotDataFilter>(robot, this->world_model->getGlobalFrame()));

        frequency_plot.getLine(fmt::format("Robot {}", robot.id));
    }

    ball_velocity_plot.getLine("Ball velocity");

    this->world_model->pushNewBallInfo(transform::BallInfo());
    this->world_model->pushNewGameState(GameState::NORMAL, time::now());
    this->world_model->setGoalieId(this->goalie);
}

GameDataProvider::~GameDataProvider() = default;

void GameDataProvider::setup() {
    // give marker service the real world model reference
    ms.setRealWorldmodel(this->world_model);

    // TODO check later

    ssl_interface.setVisionCallback(
        [this](auto&& data) { this->data_processor->appendData(std::forward<decltype(data)>(data)); });
    ssl_interface.setFieldCallback(
        [this](auto&& data) { this->data_processor->appendData(std::forward<decltype(data)>(data)); });
    ssl_interface.setGameControllerCallback(
        [this](auto&& data) { this->data_processor->appendData(std::forward<decltype(data)>(data)); });
    robot_interface.addCallback(this->feedback_callback);
    robot_interface.addCommandCallback(this->command_callback);

    size_t marker_id = 0;
    for (const RobotIdentifier& robot : this->world_model->getPossibleRobots()) {
        this->world_model->removeRobotFromField(robot);
        marker::Robot r{{robot.getFrame(), 0, 0, 0}, robot, "robots", marker_id++};
        r.setColor(robot.isAlly() ? marker::Color::BLUE() : marker::Color::YELLOW());
        ms.displayMarker(r);
        // display robot id above
        constexpr double TEXT_HEIGHT = 0.3;
        marker::Text id_text{{robot.getFrame(), 0, 0, 0}, "robots", marker_id++};
        id_text.setText(std::to_string(robot.id));
        id_text.setHeight(TEXT_HEIGHT);
        id_text.setColor(marker::Color::WHITE());
        ms.displayMarker(id_text);
    }
    transform::TransformWithVelocity trans_with_vel;
    trans_with_vel.header.stamp = time::now();
    trans_with_vel.header.child_frame = this->world_model->getBallFrame();
    trans_with_vel.transform = transform::OUT_OF_GAME_TRANSFORM;
    trans_with_vel.velocity = {0.0, 0.0, 0.0};
    this->world_model->pushTransform(trans_with_vel);
    // required to push two times so if no transform present before there is no error
    trans_with_vel.header.stamp = time::now() + time::Duration(0, 1);
    this->world_model->pushTransform(trans_with_vel);

    marker::Ball b{this->world_model->getBallFrame(), "ball", 0};
    b.setColor(marker::Color::ORANGE());
    ms.displayMarker(b);
    // visualize ball position
    constexpr double CIRCLE_SIZE = 0.06;
    constexpr double ALPHA = 0.5;
    marker::Circle c{this->world_model->getBallFrame(), "ball", 1};
    c.setFilled(true);
    c.setRadius(CIRCLE_SIZE);
    c.setColor(marker::Color::ORANGE(ALPHA));
    ms.displayMarker(c);

    std::thread obs_thread(&GameDataProvider::startObserverThread, std::ref(*this));
    this->observer_thread.swap(obs_thread);
}

void GameDataProvider::stop() {
    this->should_run = false;
    if (this->observer_thread.joinable()) {
        this->observer_thread.join();
    }
}

void GameDataProvider::loop(std::atomic_bool& /*should_run*/) {
    this->data_processor->waitForNewData([this](auto&& data) {
        auto visitor = overload{
            [this](const ssl_interface::SSLFieldData& field_data) { this->onNewFieldData(field_data); },
            [this](const ssl_interface::SSLVisionData& vision_data) { this->onNewVisionData(vision_data); },
            [this](const ssl_interface::SSLGameControllerData& gc_data) { this->onNewGameControllerData(gc_data); },
            [this](const std::pair<RobotIdentifier, robot_interface::RobotFeedback>& robot_feedback) {
                this->onNewRobotFeedback(robot_feedback);
            },
            [this](const std::pair<uint32_t, robot_interface::RobotCommand>& robot_command) {
                this->onNewRobotCommand(robot_command);
            },
        };

        std::visit(visitor, std::forward<decltype(data)>(data));
    });
}

void GameDataProvider::onNewVisionData(const ssl_interface::SSLVisionData& data) {
    {
        // Lock the data from other while we're replacing the old data
        const std::unique_lock lock(this->vision_data_mutex);

        /// @todo change to vision time, but we have to sync first
        this->vision_data_timestamp = time::now();  // data.timestamp_capture;
        // this->ally_robots.clear();
        // this->enemy_robots.clear();
        // this->robots.clear();

        // constexpr size_t NUM_ROBOTS = 16;

        bool mirror = config_provider::ConfigProvider::getConfigStore().game_config.is_flipped;

        /// @todo refactor this code
        constexpr double RADIAN_TO_DEGREES = 180.0 / L_PI;
        for (const auto& blue_base_data : data.blue_robots) {
            ssl_interface::SSLRobotInfo blue = blue_base_data;

            if (mirror) {
                blue.transform.translation().x() *= -1.0;
                blue.transform.translation().y() *= -1.0;
                blue.transform.rotate(Eigen::Rotation2Dd(L_PI));
            }

            const auto handle = createRIDFromId<TeamColor::BLUE>(blue.id);

            if (handle.isAlly()) {
                // Info marker

                std::lock_guard lock(this->robot_infos_mutex);
                auto info_it = this->robot_infos.find(handle);
                if (info_it == this->robot_infos.end()) {
                    marker::RobotInfo info(handle);
                    info_it = this->robot_infos.insert_or_assign(handle, info).first;
                }
                marker::RobotInfo& robot_info = info_it->second;
                robot_info.setStatus("Connected", marker::Color::GREEN());
                robot_info.addParam(
                    "Vision Coordinates",
                    fmt::format("({:0.3f}, {:0.3f}, {:0.3f})", blue.transform.translation().x(),
                                blue.transform.translation().y(),
                                Eigen::Rotation2Dd(blue.transform.rotation()).angle() * RADIAN_TO_DEGREES));
                auto filter_it = this->robot_data_filters.find(handle);
                if (filter_it != this->robot_data_filters.end()) {
                    if (filter_it->second->addVisionData(
                            blue.transform,
                            config_provider::ConfigProvider::getConfigStore().game_data_provider_config.vision_delay_ms,
                            this->vision_data_timestamp)) {
                        this->world_model->pushTransform(filter_it->second->getLatestTransform());
                        this->world_model->pushAllyRobotData(handle, filter_it->second->getLatestRobotData());

                    } else if (config_provider::ConfigProvider::getConfigStore()
                                   .game_data_provider_config.show_vision_robot_position) {
                        publishRobotToWorldModel(blue, handle, "_vision");
                    }
                    robot_info.addParam("", fmt::format("{}", filter_it->second->getFilterMode()));
                    this->ms.displayMarker(robot_info);
                }
            } else {
                publishRobotToWorldModel(blue, handle);
                publishRobotData(handle);
            }
        }

        for (const auto& yellow_base_data : data.yellow_robots) {
            ssl_interface::SSLRobotInfo yellow = yellow_base_data;
            if (mirror) {
                yellow.transform.translation().x() *= -1.0;
                yellow.transform.translation().y() *= -1.0;
                yellow.transform.rotate(Eigen::Rotation2Dd(L_PI));
            }
            const auto handle = createRIDFromId<TeamColor::YELLOW>(yellow.id);
            if (handle.isAlly()) {
                // Info marker

                std::lock_guard lock(this->robot_infos_mutex);
                auto info_it = this->robot_infos.find(handle);
                if (info_it == this->robot_infos.end()) {
                    marker::RobotInfo info(handle);
                    info_it = this->robot_infos.insert_or_assign(handle, info).first;
                }
                marker::RobotInfo& robot_info = info_it->second;
                robot_info.setStatus("CONNECTED", marker::Color::GREEN());
                robot_info.addParam(
                    "Vision Coordinates",
                    fmt::format("({:0.3f}, {:0.3f}, {:0.3f})", yellow.transform.translation().x(),
                                yellow.transform.translation().y(),
                                Eigen::Rotation2Dd(yellow.transform.rotation()).angle() * RADIAN_TO_DEGREES));
                auto filter_it = this->robot_data_filters.find(handle);
                if (filter_it != this->robot_data_filters.end()) {
                    if (filter_it->second->addVisionData(
                            yellow.transform,
                            config_provider::ConfigProvider::getConfigStore().game_data_provider_config.vision_delay_ms,
                            this->vision_data_timestamp)) {
                        this->world_model->pushTransform(filter_it->second->getLatestTransform());
                        this->world_model->pushAllyRobotData(handle, filter_it->second->getLatestRobotData());

                    } else if (config_provider::ConfigProvider::getConfigStore()
                                   .game_data_provider_config.show_vision_robot_position) {
                        publishRobotToWorldModel(yellow, handle, "_vision");
                    }
                    robot_info.addParam("", fmt::format("{}", filter_it->second->getFilterMode()));
                    this->ms.displayMarker(robot_info);
                }
            } else {
                publishRobotToWorldModel(yellow, handle);
                publishRobotData(handle);
            }
        }

        std::vector<ssl_interface::SSLBallInfo> balls_copy;

        for (const auto& ball : data.balls) {
            ssl_interface::SSLBallInfo ball_copy = ball;
            if (mirror) {
                ball_copy.position.x() *= -1.0;
                ball_copy.position.y() *= -1.0;
            }
            balls_copy.push_back(ball_copy);
        }

        this->ball_filter->addVisionData(balls_copy, time::now(), data.camera_id);
        auto ball_info = this->ball_filter->getLatestBallInfo();

        if (ball_info.position && ball_info.position->second) {
            auto vel = ball_info.position->second->norm();
            const auto& ball_velocity_line = ball_velocity_plot.getLine("Ball velocity");
            this->ball_velocity_plot.addPoint(ball_velocity_line, static_cast<float>(time::now().asSec()),
                                              static_cast<float>(vel));
            this->ms.displayMarker(this->ball_velocity_plot);
        }

        this->world_model->updateBallPosition(ball_info);
        this->world_model->pushNewBallInfo(ball_info);

        // update ball in dribbler in robot filter
        for (auto& filter : this->robot_data_filters) {
            filter.second->setBallInDribbler(ball_info.state == transform::BallState::IN_ROBOT &&
                                                 ball_info.robot.has_value() && ball_info.robot.value() == filter.first,
                                             ball_info.time);
        }

        removeMissingRobots();
    }

    {
        static bool is_blue_old = config_provider::ConfigProvider::getConfigStore().game_config.is_blue;
        bool is_blue_curr = config_provider::ConfigProvider::getConfigStore().game_config.is_blue;

        if (is_blue_curr != is_blue_old) {
            LOG_INFO(this->logger, "Flipped Sides. Removing old robots from field...!");

            for (const auto& robot : this->world_model->getVisibleRobots()) {
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

    this->observer_trigger.notify_one();
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
    const std::unique_lock lock(this->gamecontroller_data_mutex);
    if (this->gamecontroller_data_timestamp != data.timestamp_issued) {
        this->gamecontroller_data_timestamp = data.timestamp_issued;
        this->current_state = nextGameState(data);
        this->world_model->pushNewGameState(current_state, time::now());
        LOG_INFO(logger, "Got new game state {}", this->current_state);
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
        v.header.child_frame = BALL_PLACEMENT_POSITION_FRAME;

        v.transform = Eigen::Translation2d(data.designated_position->x(), data.designated_position->y());

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
    marker::GoalBorder goal_border_right{
        {this->world_model->getGlobalFrame(), data.field_right_center.x(), data.field_right_center.y(), L_PI / 2},
        "SSLFieldLines",
        id++};
    goal_border_right.setColor(marker::Color::GREY());
    marker::GoalBorder goal_border_left{
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
    this->world_model->setFieldData(field_data);
}

void GameDataProvider::startObserverThread(const GameDataProvider& gdp) {
    // This funciton runs in the this->observer_thread
    while (gdp.should_run) {
        std::unique_lock lock(gdp.observer_mtx);
        gdp.observer_trigger.wait_for(lock, std::chrono::milliseconds(1000));

        // Update the Observer once
        gdp.observer->update(gdp);

        // Set (or remove) the ball-obtained position in the WorldModel
        const auto ball_obtained_pos = gdp.observer->getBallObtainedPos();
        if (ball_obtained_pos.has_value()) {
            // Only set the ballObtainedPos if it was not set so the original time is preserved
            if (gdp.world_model->getLastBallObtainPosition() == std::nullopt) {
                // convert the vector to an affine2d
                const Eigen::Affine2d ball_obtained_pos_affine =
                    Eigen::Translation2d(*ball_obtained_pos) * Eigen::Rotation2Dd(0);
                gdp.world_model->setLastBallObtainPosition(time::now(), ball_obtained_pos_affine);
            }
        } else {
            gdp.world_model->removeLastBallObtainPosition();
        }
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

void GameDataProvider::onNewRobotCommand(const std::pair<uint32_t, robot_interface::RobotCommand>& data) {
    const RobotIdentifier id(data.first, Team::ALLY);
    robot_interface::RobotCommand command = data.second;
    command.time_sent = time::now();
    auto filter_it = this->robot_data_filters.find(id);
    if (filter_it != this->robot_data_filters.end() && filter_it->second->addCommandSend(command)) {
        auto trans = filter_it->second->getLatestTransform();
        // trans.header.child_frame += "_vision";
        this->world_model->pushTransform(trans);
        this->world_model->pushAllyRobotData(id, filter_it->second->getLatestRobotData());
    }
}

void GameDataProvider::onNewRobotFeedback(std::pair<RobotIdentifier, robot_interface::RobotFeedback> feedback) {
    bool is_mirrored = config_provider::ConfigProvider::getConfigStore().game_config.is_flipped.val();

    const RobotIdentifier& id = feedback.first;
    robot_interface::RobotFeedback& data = feedback.second;

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

    if (data.has_ball.has_value()) {
        this->ball_filter->addRobotDribblerStatus(id, data.has_ball.value(), data.time_stamp);
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
        marker::RobotInfo info(feedback.first);
        this->robot_infos.emplace(id, info);
        info_it = this->robot_infos.find(id);
    }

    if (info_it != this->robot_infos.end() && data.velocity) {
        info_it->second.addParam("Velocity", fmt::format("X: {: .2f}, Y: {: .2f}, Z: {: .2f}", data.velocity->x(),
                                                         data.velocity->y(), data.velocity->z()));
        info_it->second.addParam("Has Ball", data.has_ball.value_or(false));
        this->ms.displayMarker(info_it->second);
    }

    if (info_it != this->robot_infos.end() && data.telemetry.has_value()) {
        info_it->second.addParam("Battery Voltage", data.telemetry->battery_voltage);
        info_it->second.addParam("Cap Voltage", data.telemetry->capacitor_voltage);

        if (data.telemetry_rf) {
            info_it->second.addParam("RSSI (Robot)", data.telemetry_rf->rssi_robot);
            info_it->second.addParam("RSSI (Basestation)", data.telemetry_rf->rssi_base_station);
            info_it->second.addParam("Frequency", data.telemetry_rf->frequency);

            this->frequency_plot.addPoint(feedback.first.id, static_cast<float>(time::now().asSec()),
                                          static_cast<float>(data.telemetry_rf->frequency));
            this->ms.displayMarker(this->frequency_plot);
        }

        this->ms.displayMarker(info_it->second);
    }
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
            this->world_model->removeRobotFromField(robot);
        }
    }
}

void GameDataProvider::publishBall(const ssl_interface::SSLBallInfo& info) {
    transform::TransformWithVelocity v;
    v.header.stamp = time::now();  // TODO use ssl time
    v.header.parent_frame = world_model->getGlobalFrame();
    v.header.child_frame = BALL_FRAME + "_vision";

    v.transform = Eigen::Translation2d(info.position.x(), info.position.y());

    world_model->pushTransform(std::move(v));
}

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
                    LOG_INFO(logger, "Switching to normal game state because ball moved more than {} m", ball_radius);
                    switch_to_normal = true;
                }
            }
            if (special_kick_timestamp && (static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(
                                                                   time::now() - special_kick_timestamp.value())
                                                                   .count()) > state_time)) {
                LOG_INFO(logger, "Switching to normal game state because no state change after {} seconds", state_time);
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
                    LOG_INFO(logger, "Switching to penalty enemy game state because ball moved more than {} m",
                             ball_radius);
                    switch_to_penalty = true;
                }
            }
            if (special_kick_timestamp && (static_cast<double>(std::chrono::duration_cast<std::chrono::seconds>(
                                                                   time::now() - special_kick_timestamp.value())
                                                                   .count()) > state_time)) {
                LOG_INFO(logger, "Switching to penalty enemy game state because no state change after {} seconds",
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
                    const auto next_command = data.next_command.value();
                    switch (next_command) {
                        case ssl_interface::SSLCommandType::DIRECT_FREE_BLUE:
                        case ssl_interface::SSLCommandType::INDIRECT_FREE_BLUE:
                            return GameState::FREE_KICK;
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
                    const auto next_command = data.next_command.value();
                    switch (next_command) {
                        case ssl_interface::SSLCommandType::DIRECT_FREE_YELLOW:
                        case ssl_interface::SSLCommandType::INDIRECT_FREE_YELLOW:
                            return GameState::FREE_KICK;
                        default:
                            return GameState::BALL_PLACEMENT_FORCE_START;
                    }
                } else {
                    return GameState::BALL_PLACEMENT_FORCE_START;
                }
            } else {
                if (data.designated_position) {
                    LOG_INFO(logger, "Place placement {} {}", data.designated_position->x(),
                             data.designated_position->y());
                } else {
                    LOG_INFO(logger, "Got ball placement without position");
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

}  // namespace luhsoccer::game_data_provider
