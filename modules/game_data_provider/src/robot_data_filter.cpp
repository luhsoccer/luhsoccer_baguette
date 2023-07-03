#include <utility>

#include "robot_data_filter.hpp"
#include "robot_interface/robot_interface_types.hpp"
#include "visit.hpp"
#include "config_provider/config_store_main.hpp"

namespace luhsoccer::game_data_provider {

KalmanFilter::KalmanFilter(double step_time) { this->initMatrices(step_time); }

void KalmanFilter::initMatrices(double step_time, bool force_update) {
    // clang-format off
    if(this->last_step_time == step_time && !force_update) return;
    double &t = step_time;
    constexpr double M = 1.5;
    constexpr double F = 0.22;
    this->kalman_a << 1, 0, 0, t, 0, 0,
                      0, 1, 0, 0, t, 0,
                      0, 0, 1, 0, 0, t,
                      0, 0, 0, 1-F*t/M, 0, 0,
                      0, 0, 0, 0, 1-F*t/M, 0,
                      0, 0, 0, 0, 0, 1-F*t/M;
    this->kalman_b << t*t/2, 0, 0,
                      0, t*t/2, 0,
                      0, 0, t*t/2,
                      t, 0, 0,
                      0, t, 0,
                      0, 0, t;
    // clang-format on
    this->kalman_c.setIdentity();

    // process
    this->kalman_q.setIdentity();
    auto& config = config_provider::ConfigProvider::getConfigStore().game_data_provider_config;
    this->kalman_q(0, 0) = std::pow(config.kalman_q_stddev_x, 2);      // x
    this->kalman_q(1, 1) = std::pow(config.kalman_q_stddev_y, 2);      // y
    this->kalman_q(2, 2) = std::pow(config.kalman_q_stddev_theta, 2);  // theta
    this->kalman_q(3, 3) = std::pow(config.kalman_q_stddev_vx, 2);     // vx
    this->kalman_q(4, 4) = std::pow(config.kalman_q_stddev_vy, 2);     // vy
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
    this->kalman_q(5, 5) = std::pow(config.kalman_q_stddev_vtheta, 2);  // vtheta

    // measurement
    this->kalman_r.setIdentity();
    this->kalman_r(0, 0) = std::pow(config.kalman_r_stddev_x, 2);      // x
    this->kalman_r(1, 1) = std::pow(config.kalman_r_stddev_y, 2);      // y
    this->kalman_r(2, 2) = std::pow(config.kalman_r_stddev_theta, 2);  // theta
    this->kalman_r(3, 3) = std::pow(config.kalman_r_stddev_vx, 2);     // vx
    this->kalman_r(4, 4) = std::pow(config.kalman_r_stddev_vy, 2);     // vy
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers)
    this->kalman_r(5, 5) = std::pow(config.kalman_r_stddev_vtheta, 2);  // vtheta
    this->last_step_time = step_time;
}

std::pair<Vector6d, Matrix6d> KalmanFilter::predictStepCommand(Vector6d robot_state, Matrix6d kalman_p,
                                                               const Eigen::Vector3d& velocity, double step_time) {
    this->initMatrices(step_time);
    // cmd_before = robot_state.tail(3);
    Eigen::Vector3d vel_local = robot_state.tail(3);
    Eigen::Matrix2d rotation_matrix = Eigen::Rotation2Dd(robot_state.z()).toRotationMatrix();
    vel_local.head(2) = rotation_matrix.inverse() * vel_local.head(2);
    Eigen::Vector3d vel_diff = velocity - vel_local;

    const auto& config_gdp = config_provider::ConfigProvider::getConfigStore().game_data_provider_config;

    // i dont know why but the acceleration component relative to the vel_diff has to be divided by 2
    /// @todo investigate this
    Eigen::Vector3d acc = static_cast<double>(config_gdp.kalman_k_v) * vel_local +
                          static_cast<double>(config_gdp.kalman_k_p) / step_time / 2 * vel_diff;

    auto limit_acc = [](double& diff, double value_before, double increase_max, double decrease_max) {
        if (diff * value_before > 0.0) {
            diff *= std::min(1.0, increase_max / abs(diff));
        } else {
            diff *= std::min(1.0, decrease_max / abs(diff));
        }
    };
    const auto& config = config_provider::ConfigProvider::getConfigStore().local_planner_components_config;
    limit_acc(acc.x(), vel_local.x(), config.robot_acc_max_x * config_gdp.kalman_acc_scale,
              config.robot_brk_max_x * config_gdp.kalman_acc_scale);
    limit_acc(acc.y(), vel_local.y(), config.robot_acc_max_y * config_gdp.kalman_acc_scale,
              config.robot_brk_max_y * config_gdp.kalman_acc_scale);
    limit_acc(acc.z(), vel_local.z(), config.robot_acc_max_theta * config_gdp.kalman_acc_scale,
              config.robot_brk_max_theta * config_gdp.kalman_acc_scale);

    // convert back to global
    acc.head(2) = rotation_matrix * acc.head(2);
    robot_state = this->kalman_a * robot_state + this->kalman_b * acc;
    kalman_p = this->kalman_a * kalman_p * this->kalman_a.transpose() + this->kalman_q;
    if (robot_state[2] > L_PI) {
        robot_state[2] -= 2 * L_PI;
    } else if (robot_state[2] < -L_PI) {
        robot_state[2] += 2 * L_PI;
    }
    // auto limit_velocity = [](double& value, double max) { value *= std::min(1.0, max / abs(value)); };
    // limit_velocity(robot_state[3], config.robot_vel_max_x);
    // limit_velocity(robot_state[4], config.robot_vel_max_y);
    // limit_velocity(robot_state[5], config.robot_vel_max_theta);
    return {robot_state, kalman_p};
}

std::pair<Vector6d, Matrix6d> KalmanFilter::predictStepFeedback(Vector6d robot_state, Matrix6d kalman_p,
                                                                const Eigen::Vector3d& velocity, double step_time) {
    this->initMatrices(step_time);
    // Eigen::Vector3d vel_diff = velocity - robot_state.tail(3);

    // auto limit_diff = [](double& diff, double value_before, double increase_max, double decrease_max) {
    //     if (diff * value_before > 0.0) {
    //         diff *= std::min(1.0, increase_max / abs(diff));
    //     } else {
    //         diff *= std::min(1.0, decrease_max / abs(diff));
    //     }
    // };
    // const auto& config = config_provider::ConfigProvider::getConfigStore().local_planner_components_config;
    // limit_diff(vel_diff.x(), robot_state.tail(3).x(), config.robot_acc_max_x * step_time,
    //            config.robot_brk_max_x * step_time);
    // limit_diff(vel_diff.y(), robot_state.tail(3).y(), config.robot_acc_max_y * step_time,
    //            config.robot_brk_max_y * step_time);
    // limit_diff(vel_diff.z(), robot_state.tail(3).z(), config.robot_acc_max_theta * step_time,
    //            config.robot_brk_max_theta * step_time);
    Eigen::Vector3d acc = (velocity - robot_state.tail(3)) / step_time;
    robot_state = this->kalman_a * robot_state + this->kalman_b * acc;
    kalman_p = this->kalman_a * kalman_p * this->kalman_a.transpose() + this->kalman_q;
    if (robot_state[2] > L_PI) {
        robot_state[2] -= 2 * L_PI;
    } else if (robot_state[2] < -L_PI) {
        robot_state[2] += 2 * L_PI;
    }
    // auto limit_velocity = [](double& value, double max) { value *= std::min(1.0, max / abs(value)); };
    // limit_velocity(robot_state[3], config.robot_vel_max_x);
    // limit_velocity(robot_state[4], config.robot_vel_max_y);
    // limit_velocity(robot_state[5], config.robot_vel_max_theta);
    return {robot_state, kalman_p};
}
std::pair<Vector6d, Matrix6d> KalmanFilter::correctionStepVision(const Vector6d& robot_state, const Matrix6d& kalman_p,
                                                                 const Vector6d& measurement) {
    Matrix6d kalman_h = kalman_p * this->kalman_c.transpose() *
                        (this->kalman_c * kalman_p * this->kalman_c.transpose() + this->kalman_r).inverse();
    Vector6d robot_state_diff = (measurement - this->kalman_c * robot_state);
    if (robot_state_diff.z() > L_PI) {
        robot_state_diff.z() -= 2 * L_PI;
    } else if (robot_state_diff.z() < -L_PI) {
        robot_state_diff.z() += 2 * L_PI;
    }
    Vector6d corrected_robot_state = robot_state + kalman_h * robot_state_diff;  // D = 0
    Matrix6d corrected_robot_p = (Matrix6d::Identity() - kalman_h * this->kalman_c) * kalman_p;
    if (corrected_robot_state[2] > L_PI) {
        corrected_robot_state[2] -= 2 * L_PI;
    } else if (corrected_robot_state[2] < -L_PI) {
        corrected_robot_state[2] += 2 * L_PI;
    }
    return {corrected_robot_state, corrected_robot_p};
}

std::ostream& operator<<(std::ostream& out, const RobotFilterMode value) {
    static std::map<RobotFilterMode, std::string> strings;
    if (strings.size() == 0) {
        // adding string name to map if it's the first time the function is called
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define INSERT_ELEMENT(p) strings[p] = #p
        INSERT_ELEMENT(RobotFilterMode::OUT_OF_FIELD);
        INSERT_ELEMENT(RobotFilterMode::VISION_WITH_FEEDBACK);
        INSERT_ELEMENT(RobotFilterMode::VISION_WITH_COMMAND);
        INSERT_ELEMENT(RobotFilterMode::VISION);
        INSERT_ELEMENT(RobotFilterMode::ROBOT_FEEDBACK);
#undef INSERT_ELEMENT
    }

    return out << strings[value];
}

RobotDataFilter::RobotDataFilter(const RobotIdentifier& id, std::string global_frame, time::Duration invalid_interval,
                                 double rate)
    : rate(rate),
      id(id),
      mode(RobotFilterMode::OUT_OF_FIELD),
      invalid_interval(invalid_interval),
      global_frame(std::move(global_frame)) {}

void RobotDataFilter::updateMode(time::TimePoint time) {
    if (time == time::TimePoint(0)) time = time::now();
    RobotFilterMode mode_before = this->mode;
    bool valid_feedback = this->last_feedback_time_and_type.has_value() &&
                          (time - this->last_feedback_time_and_type->first) < this->invalid_interval;
    // check if feedback with position is not to old
    if (valid_feedback && this->last_feedback_time_and_type->second) {
        this->mode = RobotFilterMode::ROBOT_FEEDBACK;
    } else if (this->last_vision_time.has_value() &&  // check if vision data received and still valid
               (time - this->last_vision_time.value()) < this->invalid_interval) {
        // check if feedback is valid
        if (valid_feedback) {
            this->mode = RobotFilterMode::VISION_WITH_FEEDBACK;
        } else if (this->last_command_time.has_value() &&  // check if valid command has been sent
                   (time - this->last_command_time.value()) < this->invalid_interval) {
            this->mode = RobotFilterMode::VISION_WITH_COMMAND;
        } else {
            this->mode = RobotFilterMode::VISION;
        }
    } else {
        this->mode = RobotFilterMode::OUT_OF_FIELD;
        this->latest_info.transform = std::nullopt;
        this->latest_info.velocity = std::nullopt;
    }
    this->latest_info.on_field = {this->mode != RobotFilterMode::OUT_OF_FIELD, time};
    if (mode_before != this->mode)
        LOG_DEBUG(this->logger, "Switching RobotFilter for {} from {} to {} .", this->id, mode_before, this->mode);
}

bool RobotDataFilter::setBallInDribbler(bool ball_in_dribbler, time::TimePoint time) {
    if (time == time::TimePoint(0)) time = time::now();
    if (this->latest_info.has_ball->second > time) return false;
    this->latest_info.has_ball = {ball_in_dribbler, time};
    return true;
}

bool RobotDataFilter::addVisionData(const Eigen::Affine2d& transform, double process_delay_ms, time::TimePoint time) {
    const std::lock_guard<std::mutex> lock(this->mtx);
    if (time == time::TimePoint(0)) time = time::now();
    time::TimePoint vision_time_before;
    if (this->last_vision_time.has_value()) vision_time_before = this->last_vision_time.value();
    this->last_vision_time = time;
    this->updateMode(time);
    constexpr double RESET_DISTANCE = 0.4;
    if (this->kalman_state.has_value() &&
        ((this->kalman_state->head(2) - transform.translation()).norm() > RESET_DISTANCE ||
         this->kalman_state->hasNaN())) {
        LOG_WARNING(this->logger, "Resetting filter of {}", this->id);
        this->kalman_state = std::nullopt;
        this->kalman_p = std::nullopt;
    }
    if (this->mode == RobotFilterMode::VISION || this->mode == RobotFilterMode::VISION_WITH_COMMAND ||
        this->mode == RobotFilterMode::VISION_WITH_FEEDBACK) {
        // check if this timestamp is newer than latest saved transform stamp
        if (!this->latest_info.transform.has_value() || this->latest_info.transform->second < time) {
            // check if feedback data exists and is valid, if so do not override
            if (this->mode == RobotFilterMode::VISION) {
                this->latest_info.transform = {transform, time::now()};
                return true;
            } else {
                // apply commands on prediction
                constexpr double S_TO_MS_FACTOR = 1000.0;
                time::TimePoint vision_capture_time =
                    time -
                    time::Duration(
                        (process_delay_ms +
                         config_provider::ConfigProvider::getConfigStore().game_data_provider_config.vision_delay_ms) /
                        S_TO_MS_FACTOR);
                if (!this->kalman_state.has_value() || !this->kalman_p.has_value()) {
                    this->kalman_state = {transform.translation().x(),
                                          transform.translation().y(),
                                          Eigen::Rotation2Dd(transform.rotation()).angle(),
                                          0.0,
                                          0.0,
                                          0.0};
                    this->kalman_p = Matrix6d();
                    this->kalman_p->setIdentity();
                    constexpr double INITIAL_COVARIANCE = 0.05;
                    this->kalman_p.value()(0, 0) = INITIAL_COVARIANCE;
                    this->kalman_p.value()(1, 1) = INITIAL_COVARIANCE;
                    this->kalman_p.value()(2, 2) = INITIAL_COVARIANCE;
                    this->last_vision_position = kalman_state->head(3);
                    this->latest_info.transform = {transform, time::now()};
                    LOG_DEBUG(this->logger, "Initializing filter states for {}", this->id);
                    return true;

                } else if (this->mode == RobotFilterMode::VISION_WITH_FEEDBACK) {
                    while (this->velocity_feedback_queue.size() > 0) {
                        const auto& [time_vel, velocity] = this->velocity_feedback_queue.front();
                        if (time_vel > vision_capture_time) break;
                        auto res = this->kalman_filter.predictStepFeedback(
                            this->kalman_state.value(), this->kalman_p.value(), velocity, 1 / this->rate);
                        this->kalman_state = res.first;
                        this->kalman_p = res.second;
                        this->velocity_feedback_queue.pop_front();
                    }
                } else if (this->mode == RobotFilterMode::VISION_WITH_COMMAND) {
                    while (this->velocity_command_queue.size() > 0) {
                        const auto& [time_command, velocity] = this->velocity_command_queue.front();
                        if (time_command > vision_capture_time) break;
                        auto res = this->kalman_filter.predictStepCommand(
                            this->kalman_state.value(), this->kalman_p.value(), velocity, 1 / this->rate);
                        this->kalman_state = res.first;
                        this->kalman_p = res.second;
                        this->velocity_command_queue.pop_front();
                    }
                }
                Eigen::Vector3d vision_velocity = {0, 0, 0};
                Eigen::Vector3d vision_position = {transform.translation().x(), transform.translation().y(),
                                                   Eigen::Rotation2Dd(transform.rotation()).angle()};
                if (this->last_vision_position.has_value() && this->last_vision_time.value() != vision_time_before) {
                    vision_velocity.head(2) =
                        (vision_position.head(2) - last_vision_position->head(2)) /
                        time::Duration(this->last_vision_time.value() - vision_time_before).asSec();
                    double rotation_diff = vision_position.z() - last_vision_position->z();
                    if (rotation_diff > L_PI) {
                        rotation_diff -= 2 * L_PI;
                    } else if (rotation_diff < -L_PI) {
                        rotation_diff += 2 * L_PI;
                    }
                    vision_velocity.z() =
                        rotation_diff / time::Duration(this->last_vision_time.value() - vision_time_before).asSec();
                }
                if (config_provider::ConfigProvider::getConfigStore().game_data_provider_config.kalman_correction) {
                    Vector6d measurement;
                    measurement.head(3) = vision_position;
                    measurement.tail(3) = vision_velocity;
                    auto res = this->kalman_filter.correctionStepVision(this->kalman_state.value(),
                                                                        this->kalman_p.value(), measurement);
                    this->kalman_state = res.first;
                    this->kalman_p = res.second;
                }

                this->last_vision_position = vision_position;
                return false;
            }
        }
    }

    return false;
}

bool RobotDataFilter::addFeedbackData(const robot_interface::RobotFeedback& feedback) {
    const std::lock_guard<std::mutex> lock(this->mtx);
    if (this->last_feedback_time_and_type.has_value() &&
        this->last_feedback_time_and_type->first >= feedback.time_stamp)
        return false;
    this->last_feedback_time_and_type = {feedback.time_stamp, feedback.position.has_value()};
    this->updateMode(feedback.time_stamp);

    // check if feedback has position and this position is newer
    if (this->mode == RobotFilterMode::ROBOT_FEEDBACK &&
        (!this->latest_info.transform.has_value() || this->latest_info.transform->second < feedback.time_stamp) &&
        feedback.position.has_value()) {
        this->latest_info.transform = {Eigen::Translation2d(feedback.position->x(), feedback.position->y()) *
                                           Eigen::Rotation2Dd(feedback.position->z()),
                                       time::now()};
    }

    if (feedback.velocity.has_value() &&
        (!this->latest_info.velocity.has_value() || this->latest_info.velocity->second < feedback.time_stamp)) {
        if (this->mode == RobotFilterMode::ROBOT_FEEDBACK) {
            this->latest_info.velocity = {feedback.velocity.value(), feedback.time_stamp};
        } else {
            // velocity in local coordinate system

            Eigen::Vector2d vel_in_global_coordinate_system =
                Eigen::Rotation2Dd(this->latest_info.transform->first.rotation()).toRotationMatrix() *
                feedback.velocity->head(2);
            this->velocity_feedback_queue.push_back(
                {feedback.time_stamp,
                 {vel_in_global_coordinate_system.x(), vel_in_global_coordinate_system.y(), feedback.velocity->z()}});
            Vector6d predict_state;
            Matrix6d predict_p;

            if (this->kalman_state.has_value() && kalman_p.has_value()) {
                predict_state = this->kalman_state.value();
                predict_p = this->kalman_p.value();

                for (const auto& [time, velocity] : this->velocity_feedback_queue) {
                    auto res =
                        this->kalman_filter.predictStepFeedback(predict_state, predict_p, velocity, 1 / this->rate);
                    predict_state = res.first;
                    predict_p = res.second;
                }
                this->latest_info.transform = {
                    Eigen::Translation2d(predict_state.x(), predict_state.y()) * Eigen::Rotation2Dd(predict_state.z()),
                    feedback.time_stamp};
                this->latest_info.velocity = {predict_state.tail(3), feedback.time_stamp};
            }
        }
    }

    if (feedback.has_ball.has_value() &&
        (!this->latest_info.has_ball.has_value() || this->latest_info.has_ball->second < feedback.time_stamp)) {
        this->latest_info.has_ball = {feedback.has_ball.value(), feedback.time_stamp};
    }

    if (feedback.telemetry.has_value() &&
        (!this->latest_info.cap_voltage.has_value() || this->latest_info.cap_voltage->second < feedback.time_stamp)) {
        this->latest_info.cap_voltage = {feedback.telemetry->capacitor_voltage, feedback.time_stamp};
    }
    return this->kalman_state.has_value() && kalman_p.has_value();
}

bool RobotDataFilter::addCommandSend(const robot_interface::RobotCommand& command) {
    const std::lock_guard<std::mutex> lock(this->mtx);
    this->last_command_time = command.time_sent;
    this->updateMode(command.time_sent);

    if (this->mode == RobotFilterMode::VISION_WITH_COMMAND) {
        auto visitor = overload{
            [this, &command](const robot_interface::RobotVelocityControl& velocity) {
                //  Eigen::Vector2d vel_in_global_coordinate_system =
                //      Eigen::Rotation2Dd(this->latest_info.transform->first.rotation()).toRotationMatrix() *
                //      velocity.desired_velocity.head(2);
                this->velocity_command_queue.push_back(
                    {command.time_sent,
                     {velocity.desired_velocity.x(), velocity.desired_velocity.y(), velocity.desired_velocity.z()}});

                Vector6d predict_state;
                Matrix6d predict_p;

                if (this->kalman_state.has_value() && kalman_p.has_value()) {
                    predict_state = this->kalman_state.value();
                    predict_p = this->kalman_p.value();

                    for (const auto& [time, velocity] : this->velocity_command_queue) {
                        auto res =
                            this->kalman_filter.predictStepCommand(predict_state, predict_p, velocity, 1 / this->rate);
                        predict_state = res.first;
                        predict_p = res.second;
                    }

                    this->latest_info.transform = {Eigen::Translation2d(predict_state.x(), predict_state.y()) *
                                                       Eigen::Rotation2Dd(predict_state.z()),
                                                   command.time_sent};
                    this->latest_info.velocity = {predict_state.tail(3), command.time_sent};
                    return true;
                } else {
                    return false;
                }
            },
            [](const robot_interface::RobotPositionControl& /*position*/) {
                /// @todo implement position control
                return false;
            }};
        if (command.move_command.has_value()) std::visit(visitor, command.move_command.value());
        return true;
    }
    return false;
}

transform::TransformWithVelocity RobotDataFilter::getLatestTransform() {
    const std::lock_guard<std::mutex> lock(this->mtx);
    transform::TransformWithVelocity res;
    res.header.child_frame = this->id.getFrame();
    if (this->latest_info.transform.has_value() &&
        (time::now() - this->latest_info.transform->second) < this->invalid_interval) {
        res.transform = this->latest_info.transform->first;
        res.header.stamp = this->latest_info.transform->second;
    } else {
        res.header.stamp = time::now();
    }

    if (this->latest_info.velocity.has_value() &&
        (time::now() - this->latest_info.velocity->second) < this->invalid_interval) {
        res.velocity = this->latest_info.velocity->first;
    }
    return res;
}

transform::AllyRobotData RobotDataFilter::getLatestRobotData() {
    const std::lock_guard<std::mutex> lock(this->mtx);
    transform::AllyRobotData res;
    res.time = time::now();
    res.on_field = this->latest_info.on_field.has_value() &&
                   (res.time - this->latest_info.on_field->second) < this->invalid_interval &&
                   this->latest_info.on_field->first;
    res.ball_in_dribbler = this->latest_info.has_ball.has_value() &&
                           (res.time - this->latest_info.has_ball->second) < this->invalid_interval &&
                           this->latest_info.has_ball->first;
    if (this->latest_info.cap_voltage.has_value() &&
        (res.time - this->latest_info.cap_voltage->second) < this->invalid_interval) {
        res.cap_voltage = this->latest_info.cap_voltage->first;
    }
    return res;
}

}  // namespace luhsoccer::game_data_provider