
#include "game_data_provider/game_data_provider.hpp"
#include "observer/static_observer.hpp"
#include "observer/static_observer_position.hpp"
#include "observer/continuous_observer.hpp"
#include "observer/utility.hpp"
#include "transform_helper/world_model_helper.hpp"
#include "config_provider/config_store_main.hpp"

namespace luhsoccer::observer {

Observer::Observer(std::weak_ptr<const transform::WorldModel> world_model)
    : world_model(std::move(world_model)), cs(config_provider::ConfigProvider::getConfigStore()), logger("observer") {}

Observer::Observer() : world_model(), cs(config_provider::ConfigProvider::getConfigStore()), logger("observer") {}

void Observer::setWorldModel(std::weak_ptr<const transform::WorldModel> wm) { this->world_model = std::move(wm); }

void Observer::update(const game_data_provider::GameDataProvider& gdp) {
    // lock pointer to WorldModel
    std::shared_ptr<const transform::WorldModel> world_model_shared = this->world_model.lock();

    if (!world_model_shared) {
        LOG_DEBUG(this->logger, "world model invalid");
        return;
    }

    // calculate updates
    this->updateBallPosession(gdp);

    this->updateGoalProbability(world_model_shared);

    this->updateBestPassReceiver(world_model_shared);

    this->updateEnemyThreatLevel(world_model_shared);

    this->updateStrategyType(world_model_shared);

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

    bool ball_goal_prob_set = false;

    for (const auto identifier : world_model_shared->getVisibleRobots<Team::ALLY>()) {
        const auto goal_score =
            calculation::calculateGoalProbability(transform::RobotHandle(identifier, world_model_shared));

        if (!goal_score.has_value()) {
            // LOG_DEBUG(this->logger, "Could not update goal probability of ally");
            continue;
        }

        this->data_storage_buffer.getBackBuffer().setGoalProbability(identifier, *goal_score);
        if (ball_holder.has_value() && ball_holder->getID() == identifier) {
            this->data_storage_buffer.getBackBuffer().setBallGoalProbability(*goal_score);
            ball_goal_prob_set = true;
        }
    }

    // if the ball-goal-probability was not set previously, calculate & set it again
    if (!ball_goal_prob_set) {
        const auto ball_frame = world_model_shared->getBallFrame();
        const auto goal_prob =
            calculation::calculateGoalProbability(transform::Position(ball_frame), Team::ALLY, world_model_shared);
        if (goal_prob.has_value()) {
            this->data_storage_buffer.getBackBuffer().setBallGoalProbability(*goal_prob);
        }
    }
}

void Observer::updateBallPosession(const game_data_provider::GameDataProvider& gdp) {
    const auto new_ball_holder = calculation::calculateBallPosession(gdp);

    this->data_storage_buffer.getBackBuffer().setBallHolder(new_ball_holder);

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
                this->data_storage_buffer.getBackBuffer().setBallTouchingForbiddenRobot(new_ball_holder->handle);
            } else {
                if (this->double_touch_last_ball_holder->getID() != new_ball_holder->handle.getID()) {
                    // the robot who currently touches the ball is not the robot who last touched the ball
                    // -> All Robos can touch the ball again
                    this->data_storage_buffer.getBackBuffer().setBallTouchingForbiddenRobot(std::nullopt);
                    this->double_touch_last_ball_holder = std::nullopt;
                    was_free_kick = false;
                }
            }
        }
    }

    if (game_state == transform::GameState::HALT || game_state == transform::GameState::STOP) {
        was_free_kick = false;
        this->data_storage_buffer.getBackBuffer().setBallTouchingForbiddenRobot(std::nullopt);
        this->double_touch_last_ball_holder = std::nullopt;
    }

    /* ----------- UPDATE LAST BALL TOUCHER ----------- */
    if (new_ball_holder.has_value() && new_ball_holder->ball_obtained_pos.has_value()) {
        this->double_touch_last_ball_holder = new_ball_holder->handle;
        this->data_storage_buffer.getBackBuffer().setLastBallToucher(new_ball_holder->handle);
    }
}

void Observer::updateStrategyType(const std::shared_ptr<const transform::WorldModel>& world_model_shared) {
    static auto static_old_time = time::now();
    const auto current_time = time::now();

    const auto delta = static_cast<time::Duration>(current_time - static_old_time);

    static_old_time = current_time;

    const auto new_ball_posession = this->data_storage_buffer.getBackBuffer().getBallControllingTeam();
    const auto old_ball_posession = this->data_storage_buffer.getFrontBuffer().getBallControllingTeam();

    // no update if currently no team has the ball (old state should be preserved)
    if (!new_ball_posession.has_value()) return;

    // if the ball posession changed, reset the timer
    if (new_ball_posession != old_ball_posession) {
        this->ball_posession_timer = 0;
    } else {
        this->ball_posession_timer += delta;
    }

    if (this->ball_posession_timer.asSec() > cs.observer_config.strategy_type_threshhold) {
        if (new_ball_posession == Team::ENEMY) {
            this->data_storage_buffer.getBackBuffer().setStrategyType(StrategyType::DEFENSIVE);

        } else {
            this->data_storage_buffer.getBackBuffer().setStrategyType(StrategyType::OFFENSIVE);
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

        if (!threat_level.has_value()) {
            // LOG_DEBUG(this->logger, "Could not update threat level of enemy");
            continue;
        }

        this->data_storage_buffer.getBackBuffer().setThreatLevel(identifier, *threat_level);

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

}  // namespace luhsoccer::observer