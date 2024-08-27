#include "ball_filter.hpp"

#include "config_provider/config_store_main.hpp"
#include "config/game_config.hpp"
#include "config/ball_filter_config.hpp"

namespace luhsoccer::game_data_provider {

LightBarrierSM::LightBarrierSM()
    : debounce_time(config_provider::ConfigProvider::getConfigStore().ball_filter_config.light_barrier_debounce_time) {}

void LightBarrierSM::update() {
    switch (state) {
        case State::WAIT_FOR_BALL:
            if (light_barrier) {
                next_event = time::now() + debounce_time;
                state = State::TRIGGERED_DEBOUNCE;
            }
            break;
        case State::TRIGGERED_DEBOUNCE:
            if (time::now() > next_event) {
                state = State::TRIGGERED;
            } else if (!light_barrier) {
                state = State::WAIT_FOR_BALL;
            }
            break;
        case State::TRIGGERED:
            if (!light_barrier) {
                next_event = time::now() + debounce_time;
                state = State::LOST_DEBOUNCE;
            }
            break;
        case State::LOST_DEBOUNCE:
            if (time::now() > next_event) {
                state = State::WAIT_FOR_BALL;
            } else if (light_barrier) {
                state = State::TRIGGERED;
            }
            break;
    }
}

bool LightBarrierSM::hasBall() { return state == State::TRIGGERED || state == State::LOST_DEBOUNCE; }

BallFilter::BallFilter(std::shared_ptr<transform::WorldModel> world_model, marker::MarkerService& marker_service)
    : world_model(world_model),
      last_x_values(config_provider::ConfigProvider::getConfigStore().ball_filter_config.velocity_average_window_size),
      last_y_values(config_provider::ConfigProvider::getConfigStore().ball_filter_config.velocity_average_window_size),
      marker_service(marker_service),
      ball_velocity_plot("Ball velocity") {
    for (auto& robot : world_model->getPossibleRobots<Team::ALLY>()) {
        light_barrier_states.insert({robot, LightBarrierSM()});
    }

    ball_velocity_plot.getLine("Ball velocity");
    this->world_model->pushNewBallInfo(transform::BallInfo());

    marker::Ball b{this->world_model->getBallFrame(), "ball", 0};
    b.setColor(marker::Color::ORANGE());
    marker_service.displayMarker(b);
    // visualize ball position
    constexpr double CIRCLE_SIZE = 0.5;
    constexpr double ALPHA = 0.9;
    marker::Circle c{this->world_model->getBallFrame(), "ball", 1};
    c.setFilled(false);
    c.setThickness(0.03);
    c.setRadius(CIRCLE_SIZE);
    c.setHeight(0.1);
    c.setColor(marker::Color::ORANGE(ALPHA));
    marker_service.displayMarker(c);
};

void BallFilter::setup(event_system::EventSystem& system) {
    system.registerEventHandler<vision_processor::NewProcessedVisionDataEvent>(
        [this](const event_system::EventContext<vision_processor::NewProcessedVisionDataEvent>& ctx) {
            onNewProcessedVisionData(ctx.event);
        },
        true);

    system.registerEventHandler<robot_interface::RobotFeedbackReceivedEvent>(
        [this](const event_system::EventContext<robot_interface::RobotFeedbackReceivedEvent>& ctx) {
            onNewRobotFeedback(ctx.event);
        },
        true);
}

void BallFilter::onNewProcessedVisionData(const vision_processor::NewProcessedVisionDataEvent& event) {
    bool mirror = config_provider::ConfigProvider::getConfigStore().game_config.is_flipped;
    auto ball = event.data.ball;

    if (ball) {
        if (mirror) {
            ball->translation().x() *= -1.0;
            ball->translation().y() *= -1.0;
        }

        publishVisionBall(*ball);
        std::lock_guard lock(mutex);

        auto delta_pos = last_ball_position.translation() - ball->translation();
        auto time_diff = time::Duration((time::now() - last_ball_position_time.load())).asSec();
        auto vel_x = abs(delta_pos.x() / time_diff);
        auto vel_y = abs(delta_pos.y() / time_diff);

        double max_vel = config_provider::ConfigProvider::getConfigStore().ball_filter_config.velocity_threshold;

        if (vel_x > max_vel || vel_y > max_vel) {
            // logger.warning("Ball velocity is too high: x: {}, y: {}. Time: {} sec", vel_x, vel_y, time_diff);
            return;
        }

        last_x_values.push(vel_x);
        last_y_values.push(vel_y);

        last_ball_position = *ball;
        last_ball_position_time = time::now();

        findEnemiesHoldingBall();
    }
    publishBallInfo();
}

void BallFilter::onNewRobotFeedback(const robot_interface::RobotFeedbackReceivedEvent& event) {
    {
        std::lock_guard lock(mutex);
        if (event.feedback.has_ball.has_value() && *event.feedback.has_ball &&
            isLightBarrierPlausible(event.id, time::now())) {
            light_barrier_states[event.id].light_barrier = true;
        } else if (event.feedback.has_ball.has_value() && !(*event.feedback.has_ball)) {
            light_barrier_states[event.id].light_barrier = false;
        }
        light_barrier_states[event.id].update();

        if (light_barrier_states[event.id].hasBall() && last_ball_holder == event.id) {
            last_ball_holder_time = time::now();
        } else if (light_barrier_states[event.id].hasBall() && last_ball_holder != event.id) {
            logger.info("Robot {} got the ball", event.id);
            last_ball_holder_position = std::nullopt;
            last_ball_holder = event.id;
            last_ball_holder_time = time::now();
        } else if (!light_barrier_states[event.id].hasBall() && last_ball_holder == event.id) {
            logger.info("Robot {} lost the ball", event.id);
            last_ball_holder_position_time = time::now();
            std::optional<transform::Transform> ball_transform = world_model->getTransform("ball");
            if (ball_transform) {
                last_ball_holder_position = ball_transform->transform;
            }
            last_ball_holder = RobotIdentifier::create_empty();
        }
    }
    publishBallInfo();
}

void BallFilter::publishVisionBall(const Eigen::Affine2d& info) {
    // Publish the vision ball to the world model
    world_model->pushTransform({{"ball_vision", world_model->getGlobalFrame(), time::now()}, info, {}});
}

bool BallFilter::isLightBarrierPlausible(RobotIdentifier id, time::TimePoint light_barrier_time) {
    auto ball_holder_position = transform::RobotHandle(id, world_model).getPosVec();
    auto ball_position = world_model->getBallPosition();

    if (ball_holder_position && ball_position) {
        auto distance = (ball_holder_position.value() - ball_position.value()).norm();
        time::TimePoint time_diff(light_barrier_time -
                                  std::max(last_ball_holder_time.load(), last_ball_position_time.load()));
        auto speed = distance / time_diff.asSec();
        return speed <
               (config_provider::ConfigProvider::getConfigStore().ball_filter_config.light_barrier_velocity_threshold);
    } else {
        return true;
    }
}

void BallFilter::publishBallInfo() {
    std::lock_guard lock(mutex);

    // If one of our robots has the ball, we reset the light barrier if we don't receive any feedback for 2
    // seconds
    if (last_ball_holder.isAlly() &&
        last_ball_holder_time.load() >
            (time::now() +
             time::Duration(config_provider::ConfigProvider::getConfigStore().ball_filter_config.max_timeout_time))) {
        last_ball_holder = RobotIdentifier::create_empty();
    }

    // If there is any robot (ally or enemy) holding the ball, we set the ball inside the robot. The velocity is
    // also set to the same as the robot
    if (last_ball_holder != RobotIdentifier::create_empty()) {
        auto robot_vel_opt = transform::RobotHandle(last_ball_holder, world_model).getVelocity();
        // Pos is irrelevant, since the ball is in the robot
        Eigen::Affine2d robot_pos = Eigen::Translation2d(0.0, 0.0) * Eigen::Rotation2Dd(0.0);
        Eigen::Vector3d robot_vel = robot_vel_opt.has_value() ? *robot_vel_opt : Eigen::Vector3d(0.0, 0.0, 0.0);
        transform::BallInfo ball_info{time::now(), transform::BallState::IN_ROBOT, last_ball_holder,
                                      std::make_pair(robot_pos, robot_vel)};
        publishMarker(ball_info);
        world_model->pushNewBallInfo(ball_info);
        world_model->updateBallPosition(ball_info);

    } else {  // This means there is no robot holding the ball (based on vision data or light barrier) so we use
              // the vision data
        // We calculate the average of the last x and y velocities to get the ball velocity
        auto avg = [](const CircularBuffer<double>& buffer) {
            double sum = 0;
            for (std::size_t i = 0; i < buffer.size(); i++) {
                sum += buffer.at(i);
            }

            return sum / buffer.size();
        };

        Eigen::Vector3d ball_vel = {avg(last_x_values), avg(last_y_values), 0.0};

        Eigen::Affine2d ball_pos = last_ball_position;

        // If there was a robot holding the ball, and we didn't receive any vision after that, we assume that
        // ball still lies on the spot where the robot lost the ball
        if (last_ball_holder_position.has_value() &&
            last_ball_holder_position_time.load() > last_ball_position_time.load()) {
            ball_pos = *last_ball_holder_position;
        }

        transform::BallInfo ball_info{time::now(), transform::BallState::ON_FIELD, std::nullopt,
                                      std::make_pair(ball_pos, ball_vel)};
        publishMarker(ball_info);
        world_model->pushNewBallInfo(ball_info);
        world_model->updateBallPosition(ball_info);
    }
}

void BallFilter::publishMarker(const transform::BallInfo& ball_info) {
    if (ball_info.position.has_value()) {
        auto vel = ball_info.position->second.norm();
        const auto& ball_velocity_line = ball_velocity_plot.getLine("Ball velocity");
        this->ball_velocity_plot.addPoint(ball_velocity_line, static_cast<float>(time::now().asSec()),
                                          static_cast<float>(vel));
        this->marker_service.displayMarker(this->ball_velocity_plot);
    }
}

void BallFilter::findEnemiesHoldingBall() {
    double dribbler_width = config_provider::ConfigProvider::getConfigStore().ball_filter_config.dribbler_width;
    double x_margin = config_provider::ConfigProvider::getConfigStore().ball_filter_config.x_margin;
    double behind_robot_margin =
        config_provider::ConfigProvider::getConfigStore().ball_filter_config.behind_robot_margin;
    double y_debounce_margin = config_provider::ConfigProvider::getConfigStore().ball_filter_config.y_debounce_margin;
    double x_debounce_margin = 0.05;

    for (auto& robot : world_model->getVisibleRobots<Team::ENEMY>()) {
        auto ball_pos = world_model->getTransform("ball_vision", robot.getFrame());
        if (!ball_pos.has_value()) continue;

        auto relative_pos = ball_pos->transform.translation();
        if (last_ball_holder != robot && (relative_pos.x() > -behind_robot_margin && relative_pos.x() < x_margin &&
                                          abs(relative_pos.y()) < dribbler_width / 2.0)) {
            logger.info("Robot {} got the ball", robot);
            last_ball_holder_position = std::nullopt;
            last_ball_holder = robot;
        } else if (last_ball_holder == robot &&
                   (relative_pos.x() < -behind_robot_margin || relative_pos.x() > x_margin + x_debounce_margin ||
                    abs(relative_pos.y()) > dribbler_width / 2.0 + y_debounce_margin)) {
            logger.info("Robot {} lost the ball", robot);
            last_ball_holder_position_time = time::now();

            std::optional<transform::Transform> ball_transform = world_model->getTransform("ball");
            if (ball_transform) {
                last_ball_holder_position = ball_transform->transform;
            }
            last_ball_holder = RobotIdentifier::create_empty();
        }
    }
}

}  // namespace luhsoccer::game_data_provider