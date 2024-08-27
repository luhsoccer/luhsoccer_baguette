
#include "game_data_provider/game_data_provider.hpp"
#include "observer/observer_events.hpp"
#include "observer/static_observer.hpp"
#include "observer/static_observer_position.hpp"
#include "observer/continuous_observer.hpp"
#include "transform_helper/world_model_helper.hpp"
#include "config_provider/config_store_main.hpp"
#include "config/observer_config.hpp"
#include "event_system/event_system.hpp"
#include "observer/observer_events.hpp"

namespace luhsoccer::observer {

Observer::Observer(std::weak_ptr<const transform::WorldModel> world_model, event_system::EventSystem& event_system)
    : world_model(std::move(world_model)),
      event_system(event_system),
      cs(config_provider::ConfigProvider::getConfigStore()),
      logger("observer") {}

void Observer::setWorldModel(std::weak_ptr<const transform::WorldModel> wm) { this->world_model = std::move(wm); }

void Observer::update(const game_data_provider::GameDataProvider& gdp) {
    // lock pointer to WorldModel
    std::shared_ptr<const transform::WorldModel> world_model_shared = this->world_model.lock();

    if (!world_model_shared) {
        this->logger.debug("world model invalid");
        return;
    }

    this->updateBallPosession(gdp);

    this->updateGoalProbability(world_model_shared);

    this->updateBestPassReceiver(world_model_shared);

    this->updateEnemyThreatLevel(world_model_shared);

    this->updateStrategyType(world_model_shared);

    this->checkIfRobotMoved(world_model_shared);

    this->checkForBallEvent(world_model_shared);

    this->updateBestInterceptor(world_model_shared);

    // switch buffers
    this->data_storage_buffer.switchBuffers();
}

void Observer::updateBestPassReceiver(const std::shared_ptr<const transform::WorldModel>& world_model_shared) {
    for (const auto identifier : world_model_shared->getVisibleRobots<Team::ALLY>()) {
        const auto best_pass_receiver =
            calculation::calculateBestPassReceiver(transform::RobotHandle(identifier, world_model_shared));

        if (!best_pass_receiver.has_value()) {
            continue;
        }

        this->data_storage_buffer.getBackBuffer().setBestPassReceiver(identifier, best_pass_receiver->handle,
                                                                      best_pass_receiver->score);
    }
}

void Observer::updateGoalProbability(const std::shared_ptr<const transform::WorldModel>& world_model_shared) {
    const auto ball_holder = this->data_storage_buffer.getBackBuffer().getBallCarrier();

    for (const auto identifier : world_model_shared->getVisibleRobots<Team::ALLY>()) {
        const auto goal_score =
            calculation::calculateGoalProbability(transform::RobotHandle(identifier, world_model_shared));

        this->data_storage_buffer.getBackBuffer().setGoalProbability(identifier, goal_score);
        if (ball_holder.has_value() && ball_holder->getID() == identifier) {
            this->data_storage_buffer.getBackBuffer().setBallGoalProbability(goal_score);
        }
    }
}

void Observer::updateBallPosession(const game_data_provider::GameDataProvider& gdp) {
    const auto new_ball_holder = calculation::calculateBallPosession(gdp);

    auto old_ball_holder = this->data_storage_buffer.getFrontBuffer().getBallCarrier();

    this->data_storage_buffer.getBackBuffer().setBallHolder(new_ball_holder);

    // Handle BallCarrierChanged Event
    {
        if (new_ball_holder.has_value()) {
            if (!old_ball_holder.has_value() || old_ball_holder->getID() != new_ball_holder->handle.getID()) {
                this->event_system.fireEvent(BallCarrierChangedEvent(old_ball_holder, new_ball_holder->handle));
            }

        } else {
            if (old_ball_holder.has_value()) {
                this->event_system.fireEvent(BallCarrierChangedEvent(old_ball_holder, std::nullopt));
            }
        }
    }

    /* ----------- UPDATE DOUBLE TOUCH PREVENTION ----------- */
    // only set last ball toucher if the robot really had the ball in its dribbler (only when the lastballobtained
    // position is set)

    /*
    Kickoff -> ball toucher wird zum ball-touch-forbidden robot
    Wenn anderer Roboter den ball berührt, und es kein kickoff ist, wird der ball touch forbidden robot entfernt
    */

    const std::shared_ptr<const transform::WorldModel> world_model_shared = gdp.getWorldModel();

    static bool was_free_kick = false;

    const auto last_ball_toucher = this->data_storage_buffer.getBackBuffer().getLastBallToucher();
    const auto game_state = world_model_shared->getGameState();
    if (!game_state.has_value()) {
        return;  // @todo
    }

    if ((game_state == transform::GameState::FREE_KICK ||
         game_state == transform::GameState::KICKOFF_PREP) &&  // || game_state == transform::GameState::KICKOFF
        !was_free_kick) {
        was_free_kick = true;
        this->double_touch_last_ball_holder = std::nullopt;
    }

    if (new_ball_holder.has_value() && new_ball_holder->ball_obtained_pos.has_value()) {
        if (was_free_kick) {
            if (!this->double_touch_last_ball_holder.has_value()) {
                // Robot touches the ball but bo other robot previously touched the ball -> Robot cant touch ball again
                this->data_storage_buffer.getBackBuffer().setPotentialDoubleToucher(new_ball_holder->handle);
            } else {
                if (this->double_touch_last_ball_holder->getID() != new_ball_holder->handle.getID()) {
                    // the robot who currently touches the ball is not the robot who last touched the ball
                    // -> All Robos can touch the ball again
                    this->data_storage_buffer.getBackBuffer().setPotentialDoubleToucher(std::nullopt);
                    this->double_touch_last_ball_holder = std::nullopt;
                    was_free_kick = false;
                }
            }
        }
    }

    if (game_state == transform::GameState::HALT || game_state == transform::GameState::STOP) {
        was_free_kick = false;
        this->data_storage_buffer.getBackBuffer().setPotentialDoubleToucher(std::nullopt);
        this->double_touch_last_ball_holder = std::nullopt;
    }

    /* ----------- UPDATE LAST BALL TOUCHER ----------- */
    if (new_ball_holder.has_value() && new_ball_holder->ball_obtained_pos.has_value()) {
        this->double_touch_last_ball_holder = new_ball_holder->handle;
        this->data_storage_buffer.getBackBuffer().setLastBallToucher(new_ball_holder->handle);
    }
}

void Observer::updateStrategyType(const std::shared_ptr<const transform::WorldModel>& /*world_model_shared*/) {
    static auto static_old_time = time::now();
    const auto current_time = time::now();

    const auto delta = static_cast<time::Duration>(current_time - static_old_time);

    static_old_time = current_time;

    const auto new_ball_posession = this->data_storage_buffer.getBackBuffer().getBallControllingTeam();
    const auto old_ball_posession = this->data_storage_buffer.getFrontBuffer().getBallControllingTeam();
    const auto old_strategy_type = this->data_storage_buffer.getFrontBuffer().getStrategyType();

    // no update if currently no team has the ball (old state should be preserved)
    if (!new_ball_posession.has_value()) return;

    // if the ball posession changed, reset the timer
    if (new_ball_posession != old_ball_posession) {
        this->ball_posession_timer = 0;
    } else {
        this->ball_posession_timer += delta;
    }

    if (this->ball_posession_timer.asSec() > cs.observer_config.strategy_type_threshhold) {
        StrategyType new_strategy_type{};
        if (new_ball_posession == Team::ENEMY) {
            this->data_storage_buffer.getBackBuffer().setStrategyType(StrategyType::DEFENSIVE);
            new_strategy_type = StrategyType::DEFENSIVE;
        } else {
            this->data_storage_buffer.getBackBuffer().setStrategyType(StrategyType::OFFENSIVE);
            new_strategy_type = StrategyType::OFFENSIVE;
        }

        // Todo: this is a quick fix. Definiety improve later (change strategy type to DominantTeam in whole Observer)
        if (new_strategy_type != old_strategy_type) {
            if (new_strategy_type == StrategyType::OFFENSIVE) {
                this->event_system.fireEvent(DominantTeamChangeEvent{Team::ALLY});
            } else {
                this->event_system.fireEvent(DominantTeamChangeEvent{Team::ENEMY});
            }
        }
    }

    // if (new_ball_posession == Team::ENEMY) {
    //     this->ball_posession_timer += delta;
    // } else {
    //     this->ball_posession_timer -= delta;
    // }
    //
    // Check if ball is in enemy penalty area and moves slow
    // const auto ball_pos = transform::helper::getBallPosition(*world_model_shared);
    // if (ball_pos.has_value()) {
    //     if (ball_pos->x() > 3.5 && ball_pos->y() < 1 &&
    //         ball_pos->y() > -1) {  // @todo replace with penalty area coordinates from world model
    //         this->data_storage_buffer.getBackBuffer().setStrategyType(StrategyType::DEFENSIVE);
    //         return;
    //     }
    // }

    // // clamp value
    // const double max_save_time = cs.observer_config.strategy_type_max_time;
    // this->ball_posession_timer =
    //     std::clamp(this->ball_posession_timer, time::Duration(0), time::Duration(max_save_time));

    // StrategyType type = StrategyType::OFFENSIVE;

    // const double threshhold = cs.observer_config.strategy_type_threshhold;
    // if (this->ball_posession_timer >= time::Duration(threshhold)) {
    //     type = StrategyType::DEFENSIVE;
    // }

    // // @todo Calculated based on the ball-posession (maybe timed)
    // this->data_storage_buffer.getBackBuffer().setStrategyType(type);
}

void Observer::updateEnemyThreatLevel(const std::shared_ptr<const transform::WorldModel>& world_model_shared) {
    const auto ball_holder = this->data_storage_buffer.getBackBuffer().getBallCarrier();

    for (const auto identifier : world_model_shared->getVisibleRobots<Team::ENEMY>()) {
        const auto threat_level =
            calculation::calculateThreatLevel(transform::RobotHandle(identifier, world_model_shared));

        const double threshold = this->cs.observer_config.threat_score_event_fire_threshold;
        double old_val = this->last_fired_threat_level[identifier];
        if (!(threat_level * (1.0 - threshold) < old_val && old_val < threat_level * (1.0 + threshold))) {
            this->event_system.fireEvent(ThreatLevelChangedEvent(identifier, old_val, threat_level));
            this->last_fired_threat_level[identifier] = threat_level;
        }

        this->data_storage_buffer.getBackBuffer().setThreatLevel(identifier, threat_level);

        // evaluate whether the enemy robot is pass defended
        // if the enemy has the ball he has to be completely covered. If he doesnt have the ball, the passing-line
        // to the ball_holder has to be covered
        if (!ball_holder.has_value()) continue;

        if (ball_holder->getTeam() != identifier.getTeam()) continue;

        std::optional<bool> is_pass_defended = std::nullopt;
        if (ball_holder->getID() == identifier) {
            is_pass_defended =
                calculation::isOutgoingPassDefended(transform::RobotHandle(identifier, world_model_shared));
        } else {
            is_pass_defended =
                calculation::isPassLineDefended(*ball_holder, transform::RobotHandle(identifier, world_model_shared));
        }
        if (!is_pass_defended.has_value()) continue;
        this->data_storage_buffer.getBackBuffer().setPassDefended(identifier, *is_pass_defended);
    }
}

void Observer::checkIfRobotMoved(const std::shared_ptr<const transform::WorldModel>& world_model_shared) {
    static std::unordered_map<RobotIdentifier, Eigen::Vector2d> last_positions;

    for (const auto& id : world_model_shared->getVisibleRobots()) {
        auto handle = transform::RobotHandle(id, world_model_shared);
        const auto position = transform::helper::getPosition(handle);

        if (position) {
            if (!last_positions.contains(id)) {
                // Save position for checking later
                last_positions[id] = *position;
            } else {
                double distance = (last_positions[id] - *position).norm();
                if (distance > cs.observer_config.robot_moved_event_min_distance) {
                    this->event_system.fireEvent(RobotMovedEvent(handle, distance, last_positions[id], *position));
                    last_positions[id] = *position;
                }
            }
        }
    }
}

void Observer::checkForBallEvent(const std::shared_ptr<const transform::WorldModel>& world_model_shared) {
    const float offset = 0.005;

    const auto ball_pos = transform::helper::getBallPosition(*world_model_shared);
    if (!ball_pos.has_value()) return;

    const auto corner_ally_transform = world_model_shared->getTransform(transform::field::CORNER_ALLY_LEFT);
    const auto corner_enemy_transform = world_model_shared->getTransform(transform::field::CORNER_ENEMY_RIGHT);

    if (!corner_ally_transform.has_value() || !corner_enemy_transform.has_value()) return;

    const auto corner_ally_pos = corner_ally_transform.value().transform.translation();
    const auto corner_enemy_pos = corner_enemy_transform.value().transform.translation();

    // check if ball is not in field
    if (!(ball_pos->x() > corner_ally_pos.x() - offset && ball_pos->x() < corner_enemy_pos.x() + offset &&
          ball_pos->y() < corner_ally_pos.y() + offset && ball_pos->y() > corner_enemy_pos.y() - offset)) {
        if (!this->ball_left_field) {
            event_system.fireEvent(BallLeftFieldEvent());
            this->ball_left_field = true;
            // logger.info("Ball left field");
        }
    } else if (this->ball_left_field &&
               (ball_pos->x() > corner_ally_pos.x() + offset && ball_pos->x() < corner_enemy_pos.x() - offset &&
                ball_pos->y() < corner_ally_pos.y() - offset && ball_pos->y() > corner_enemy_pos.y() + offset)) {
        this->ball_left_field = false;
    }

    // check if ball is in enemy penalty area
    const auto enemy_penalty_area_left_transform =
        world_model_shared->getTransform(transform::field::DEFENSE_AREA_CORNER_ENEMY_LEFT);
    const auto enemy_penalty_area_right_transform =
        world_model_shared->getTransform(transform::field::DEFENSE_AREA_CORNER_ENEMY_RIGHT);
    if (!enemy_penalty_area_left_transform.has_value() || !enemy_penalty_area_right_transform.has_value()) return;

    const auto enemy_penalty_area_left_pos = enemy_penalty_area_left_transform.value().transform.translation();
    const auto enemy_penalty_area_right_pos = enemy_penalty_area_right_transform.value().transform.translation();

    if (ball_pos->x() > enemy_penalty_area_left_pos.x() + offset && ball_pos->x() < corner_enemy_pos.x() - offset &&
        ball_pos->y() < enemy_penalty_area_left_pos.y() - offset &&
        ball_pos->y() > enemy_penalty_area_right_pos.y() + offset) {
        if (!this->ball_entered_enemy_penalty_area) {
            event_system.fireEvent(BallEnteredEnemyPenaltyAreaEvent());
            this->ball_entered_enemy_penalty_area = true;
            // logger.info("Ball entered enemy penalty area");
        }
    } else if (this->ball_entered_enemy_penalty_area && !(ball_pos->x() > enemy_penalty_area_left_pos.x() - offset &&
                                                          ball_pos->x() < corner_enemy_pos.x() + offset &&
                                                          ball_pos->y() < enemy_penalty_area_left_pos.y() + offset &&
                                                          ball_pos->y() > enemy_penalty_area_right_pos.y() - offset)) {
        this->ball_entered_enemy_penalty_area = false;
    }

    // check if ball is in ally penalty area
    const auto ally_penalty_area_left_transform =
        world_model_shared->getTransform(transform::field::DEFENSE_AREA_CORNER_ALLY_LEFT);
    const auto ally_penalty_area_right_transform =
        world_model_shared->getTransform(transform::field::DEFENSE_AREA_CORNER_ALLY_RIGHT);
    if (!ally_penalty_area_left_transform.has_value() || !ally_penalty_area_right_transform.has_value()) return;

    const auto ally_penalty_area_left_pos = ally_penalty_area_left_transform.value().transform.translation();
    const auto ally_penalty_area_right_pos = ally_penalty_area_right_transform.value().transform.translation();

    if (ball_pos->x() > corner_ally_pos.x() + offset && ball_pos->x() < ally_penalty_area_right_pos.x() - offset &&
        ball_pos->y() < ally_penalty_area_left_pos.y() - offset &&
        ball_pos->y() > ally_penalty_area_right_pos.y() + offset) {
        if (!this->ball_entered_ally_penalty_area) {
            event_system.fireEvent(BallEnteredAllyPenaltyAreaEvent());
            this->ball_entered_ally_penalty_area = true;
            // logger.info("Ball entered ally penalty area");
        }
    } else if (this->ball_entered_ally_penalty_area && !(ball_pos->x() > corner_ally_pos.x() - offset &&
                                                         ball_pos->x() < ally_penalty_area_right_pos.x() + offset &&
                                                         ball_pos->y() < ally_penalty_area_left_pos.y() + offset &&
                                                         ball_pos->y() > ally_penalty_area_right_pos.y() - offset)) {
        this->ball_entered_ally_penalty_area = false;
    }

    // check if ball crossed middle line
    const auto middle_line_transform = world_model_shared->getTransform(transform::field::MID_LINE_LEFT);
    if (!middle_line_transform.has_value()) return;

    const auto middle_line_pos = middle_line_transform->transform.translation();

    if (ball_pos->x() > middle_line_pos.x()) {
        if (!this->ball_in_ally_half) {
            event_system.fireEvent(BallCrossedMiddleLineEvent());
            this->ball_in_ally_half = true;
            // logger.info("Ball crossed middle line");
        }
    } else if (this->ball_in_ally_half && !(ball_pos->x() > middle_line_pos.x() - offset)) {
        this->ball_in_ally_half = false;
        event_system.fireEvent(BallCrossedMiddleLineEvent());
        // logger.info("Ball crossed middle line");
    }
}

void Observer::updateBestInterceptor(const std::shared_ptr<const transform::WorldModel>& world_model) {
    const auto& best_interceptor = calculation::calculateBestInterceptor(world_model);

    if (!best_interceptor.has_value() && !this->best_interceptor.has_value()) {
        return;
    }
    bool fire_event = false;

    if (!best_interceptor.has_value() && this->best_interceptor.has_value()) {
        this->best_interceptor = std::nullopt;
        fire_event = true;
    }

    else if (best_interceptor.has_value() && !this->best_interceptor.has_value()) {
        this->best_interceptor = best_interceptor;
        fire_event = true;
    }

    else if (best_interceptor.has_value() && this->best_interceptor.has_value()) {
        if (best_interceptor->getID() != this->best_interceptor->getID()) {
            this->best_interceptor = best_interceptor;
            fire_event = true;
        }
    }

    if (fire_event) {
        this->event_system.fireEvent(BestInterceptorChangedEvent(this->best_interceptor, best_interceptor));
    }
}

}  // namespace luhsoccer::observer