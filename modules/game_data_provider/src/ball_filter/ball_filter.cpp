#include "ball_filter.hpp"

#include "speed_filter.hpp"
#include "moving_average_filter.hpp"
#include "transform_helper/world_model_helper.hpp"

#include "config_provider/config_store_main.hpp"
#include "observer/utility.hpp"
#include <cmath>
#include <numeric>
#include <utility>
#include "static_queue.hpp"

namespace luhsoccer::game_data_provider {

BallFilter::BallFilter(std::string ball_frame, std::string global_frame,
                       std::shared_ptr<transform::WorldModel> world_model)
    : ball_frame(std::move(ball_frame)), global_frame(std::move(global_frame)), world_model(std::move(world_model)) {
    this->filters.push_back(std::make_unique<SpeedFilter>());
    this->filters.push_back(std::make_unique<MovingAverageFilter>());
}

void BallFilter::setFeedbackDribblerStatus(RobotIdentifier robot_id, bool has_ball, time::TimePoint time) {
    bool robot_not_in_vector = true;
    for (auto& data : this->ball_in_feedback_robot) {
        if (data.robot_id == robot_id) {
            data.stamp = time;
            // if (has_ball) data.last_time_in_dribbler = time;
            // if (!has_ball) data.last_time_not_in_dribbler = time;
            robot_not_in_vector = false;

            data.latest_ball_feedback.push(has_ball);
            data.ball_in_dribbler =
                std::accumulate(data.latest_ball_feedback.data().begin(), data.latest_ball_feedback.data().end(), 0.0,
                                [](int sum, bool has_ball) {
                                    if (has_ball) sum++;
                                    return sum;
                                }) > BallInDribblerData::MAX_ENTRIES / 2.0;
        }
        // else if (has_ball) {
        //     data.ball_in_dribbler = false;
        // }
    }
    if (robot_not_in_vector) {
        this->ball_in_feedback_robot.emplace_back(robot_id, has_ball, time, time::TimePoint(0), time::TimePoint(0));
    }
}

time::TimePoint BallFilter::getLatestDribblerFeedbackTime(RobotIdentifier robot_id) {
    for (auto& i : this->ball_in_feedback_robot) {
        if (i.robot_id == robot_id) {
            return i.ball_in_dribbler;
        }
    }
    return {};
}

bool BallFilter::getBallInFeedbackDribbler(time::TimePoint time) const {
    const auto& cs = luhsoccer::config_provider::ConfigProvider::getConfigStore();
    if (cs.ball_filter_config.ignore_ally_dribbler_feedback) return false;
    for (auto& i : this->ball_in_feedback_robot) {
        if (i.ball_in_dribbler) {
            if (time.asSec() - i.stamp.asSec() <= cs.ball_filter_config.ally_robot_dribbler_feedback_threshold) {
                return true;
            }
        }
        // else if (time.asSec() - i.last_time_in_dribbler.asSec() < cs.ball_filter_config.light_barrier_low_pass_time)
        // {
        //     return true;
        // }
    }
    return false;
}

void BallFilter::setVisionDribblerStatus(RobotIdentifier robot_id, bool has_ball, time::TimePoint time) {
    bool robot_not_in_vector = true;
    for (auto& i : this->ball_in_vision_robot) {
        if (std::get<0>(i) == robot_id) {
            std::get<1>(i) = has_ball;
            std::get<2>(i) = time;
            robot_not_in_vector = false;
        } else if (has_ball) {
            std::get<1>(i) = false;
        }
    }
    if (robot_not_in_vector) {
        this->ball_in_vision_robot.emplace_back(robot_id, has_ball, time);
    }
}

bool BallFilter::getBallInVisionDribbler(time::TimePoint time) const {
    const auto& cs = luhsoccer::config_provider::ConfigProvider::getConfigStore();
    for (auto& i : this->ball_in_vision_robot) {
        if (std::get<1>(i)) {
            if (time.asSec() - std::get<2>(i).asSec() < 0) {
                return true;
            }

            if (time.asSec() - std::get<2>(i).asSec() <= cs.ball_filter_config.enemy_robot_dribbler_vision_delay) {
                return true;
            }
        }
    }
    return false;
}

[[maybe_unused]] void BallFilter::setBallIsNotInRobot() {
    for (auto& i : this->ball_in_feedback_robot) {
        i.ball_in_dribbler = false;
    }
    for (auto& i : this->ball_in_vision_robot) {
        std::get<1>(i) = false;
    }
    ball_in_robot = false;
    ball_carrying_robot = std::nullopt;
}

std::optional<RobotIdentifier> BallFilter::getRobotWithBall() const {
    // create robot 1 tuple
    std::vector<BallInDribblerData> ally_robot;
    std::vector<std::tuple<RobotIdentifier, bool, time::TimePoint>> enemy_robot;

    if (getBallInFeedbackDribbler(time::now())) {
        for (auto& i : this->ball_in_feedback_robot) {
            if (i.ball_in_dribbler) {
                ally_robot.push_back(i);
                break;
            }
        }
    }
    if (getBallInVisionDribbler(time::now())) {
        for (auto& i : this->ball_in_vision_robot) {
            if (std::get<1>(i)) {
                enemy_robot.push_back(i);
                break;
            }
        }
    }

    if (ally_robot.empty() && enemy_robot.empty()) {
        return std::nullopt;
    }
    if (ally_robot.empty() && !enemy_robot.empty()) {
        return std::get<0>(enemy_robot[0]);
    }
    if (enemy_robot.empty() && !ally_robot.empty()) {
        return ally_robot[0].robot_id;
    }

    if (ally_robot[0].stamp.asSec() > std::get<2>(enemy_robot[0]).asSec()) {
        return ally_robot[0].robot_id;
    } else {
        return std::get<0>(enemy_robot[0]);
    }
}
bool BallFilter::addRobotDribblerStatus(const RobotIdentifier& robot, bool has_ball,
                                        [[maybe_unused]] time::TimePoint time) {
    setFeedbackDribblerStatus(robot, has_ball, time);
    if (!has_ball) return has_ball;
    this->ball_carrying_robot = robot;
    this->ball_in_robot = has_ball;
    return has_ball;
}

Eigen::Vector3d BallFilter::calculateBallVelocity(const Eigen::Affine2d& /*old_ball_position*/,
                                                  const Eigen::Affine2d& new_ball_position,
                                                  const time::TimePoint& /*old_stamp*/,
                                                  const time::TimePoint& new_stamp) {
    const auto ball_pos = new_ball_position.translation();

    constexpr double FIELD_WIDTH = 4.5 + 1.5;
    constexpr double FIELD_HEIGHT = 3 + 1.5;

    // out of field checks
    if (abs(ball_pos.x()) > FIELD_WIDTH || abs(ball_pos.y()) > FIELD_HEIGHT) {
        this->vel_filter_info.avg_vel = {0, 0};
        this->vel_filter_info.last_average_time = 0;
        this->vel_filter_info.last_averaged_position = {0, 0};

        for (auto& data_point : this->vel_filter_info.averaged_positions.dataMut()) {
            data_point.first = {0, 0};
            data_point.second = 0;
        }

        for (auto& data_point : this->vel_filter_info.considered_velocities.dataMut()) {
            data_point = {0, 0};
        }

        return {this->vel_filter_info.avg_vel.x(), this->vel_filter_info.avg_vel.y(), 0};
    }

    this->vel_filter_info.averaged_positions.push({ball_pos, new_stamp.asSec()});

    if (this->vel_filter_info.averaged_positions.currentIndex() == 0) {
        const auto& pos_data = this->vel_filter_info.averaged_positions.data();

        Eigen::Vector2d average_pos{0, 0};
        double average_time{0};

        for (const auto& [pos, time] : pos_data) {
            average_time += time;
            average_pos += pos;
        }

        average_pos /= static_cast<double>(this->vel_filter_info.NUM_BALLS_AVERAGED);
        average_time /= static_cast<double>(this->vel_filter_info.NUM_BALLS_AVERAGED);

        const double delta_time = average_time - this->vel_filter_info.last_average_time;
        const Eigen::Vector2d delta_pos = average_pos - this->vel_filter_info.last_averaged_position;

        const Eigen::Vector2d delta_vel = delta_pos / delta_time;

        if (delta_vel.norm() > this->vel_filter_info.MAX_VEL_TOLERANCE) {
            return {this->vel_filter_info.avg_vel.x(), this->vel_filter_info.avg_vel.y(), 0};
        }

        this->vel_filter_info.last_averaged_position = average_pos;
        this->vel_filter_info.last_average_time = average_time;

        this->vel_filter_info.considered_velocities.push(delta_vel / this->vel_filter_info.MAX_ENTRIES);

        const auto& data = this->vel_filter_info.considered_velocities.data();
        this->vel_filter_info.avg_vel = std::accumulate(data.begin(), data.end(), Eigen::Vector2d::Zero().eval());

        // @todo : edge detection: detect when the ball switches from slow to fast in order to faster react to shots
        //                         should currently be fast enough for now
        /*
        {
            // Detect if the last two velocities are significantly bigger than the average (shot detection)
            Eigen::Vector2d shot_avg_vel{0, 0};

            const auto& cons_vel = this->vel_filter_info.considered_velocities.data();
            const auto current_index = this->vel_filter_info.considered_velocities.currentIndex();
            const auto size = this->vel_filter_info.considered_velocities.size();
            shot_avg_vel += cons_vel[current_index];
            shot_avg_vel += cons_vel[(current_index + 1) % size];

            shot_avg_vel *= (static_cast<double>(size) / 2.0);

            size_t ind = current_index == 0 ? size - 1 : current_index - 1;

            // positive detect edges
            if (shot_avg_vel.norm() > this->vel_filter_info.avg_vel.norm() * 50.0 && cons_vel[ind].norm() < 0.5) {
                // Replace all entries with the average of the last two
                for (size_t i = 0; i < this->vel_filter_info.MAX_ENTRIES - 2; i++) {
                    this->vel_filter_info.considered_velocities.push(shot_avg_vel);
                }
            }
        }
        */
    }

    return {this->vel_filter_info.avg_vel.x(), this->vel_filter_info.avg_vel.y(), 0};
}

Eigen::Affine2d BallFilter::getBallAffine(const double x, const double y) {
    const Eigen::Translation2d translation = {x, y};
    const Eigen::Rotation2Dd rotation(0.0);
    Eigen::Affine2d transform = translation * rotation;
    return transform;
}

bool BallFilter::visionDataInvalid(const time::TimePoint time) const {
    if (getLatestStamp().has_value() && time < getLatestStamp().value()) {
        return true;
    }
    return false;
}

bool BallFilter::addVisionData(const std::vector<ssl_interface::SSLBallInfo>& data, const time::TimePoint time,
                               const size_t camera_id) {
    // ignore data if data vector is empty
    if (visionDataInvalid(time)) {
        return false;
    }

    // setting initial values
    if (!getLatestVisionStamp().has_value() && !getLatestBallVelocity().has_value()) {
        return setInitialVisionValues(data, time);
    }

    return setFilteredVisionData(data, time, camera_id);
}

void BallFilter::checkCameraStatus(size_t camera_id, bool accepted, time::TimePoint time) {
    // if camera_id is not in the vector add it
    // std::vector<std::tuple<size_t, bool, time::TimePoint>> camera_status;

    const auto& cs = luhsoccer::config_provider::ConfigProvider::getConfigStore();
    bool camera_id_doesnt_exists = true;
    for (auto& i : this->camera_status) {
        if (std::get<0>(i) == camera_id) {
            std::get<1>(i) = accepted;
            std::get<2>(i) = time;
            camera_id_doesnt_exists = false;
            // todo
        }

        if (std::get<2>(i).asSec() < time.asSec() - cs.ball_filter_config.camera_wait_time) {
            std::get<1>(i) = false;
        }
    }

    if (camera_id_doesnt_exists) {
        this->camera_status.emplace_back(camera_id, accepted, time);
    }
}

bool BallFilter::checkBallInvisible() {
    for (auto& i : this->camera_status) {
        if (std::get<1>(i) && (time::now() - std::get<2>(i)) < time::Duration(0.2)) {
            return false;
        }
    }
    return true;
}

void BallFilter::checkVisionDribbler(Eigen::Affine2d ball_position, time::TimePoint time) {
    const config_provider::ConfigStore& cs = luhsoccer::config_provider::ConfigProvider::getConfigStore();
    auto logger = luhsoccer::logger::Logger("ball_filter");
    Eigen::Vector2d const ball_pos = ball_position.translation();
    double max_angle = NAN;
    double max_distance = NAN;
    for (const auto& identifier : this->world_model->getVisibleRobots()) {
        // ignore ally robots if they have send a feedback within the feedback threshold and the feedback is not ignored
        if (identifier.isAlly()) {
            if (time.asSec() - getLatestDribblerFeedbackTime(identifier).asSec() <
                    cs.ball_filter_config.ally_robot_dribbler_feedback_threshold &&
                !cs.ball_filter_config.ignore_ally_dribbler_feedback) {
                // ignore ally robots if they have send a feedback within the feedback threshold
                continue;
            }
            max_angle = cs.ball_filter_config.ally_dribbler_angle;
            max_distance = cs.ball_filter_config.ally_dribbler_distance;
        } else {
            max_angle = cs.ball_filter_config.enemy_dribbler_angle;
            max_distance = cs.ball_filter_config.enemy_dribbler_distance;
        }

        const auto pos =
            transform::helper::getPositionAndRotation(transform::RobotHandle(identifier, this->world_model));
        if (!pos.has_value()) continue;

        // calculate a line between the ball and the robot und calculate the angle between the dribbler direction and
        // the line
        Eigen::Vector2d const direction_robot_to_ball = ball_pos - pos->head<2>();

        // check if distance between robot and ball is greater than the distance set in the config
        if (direction_robot_to_ball.norm() > max_distance) {  // NOLINT(cppcoreguidelines-avoid-magic-numbers)
            setVisionDribblerStatus(identifier, false, time);
            continue;
        }

        // Vector from robot center to middle point of dribbler
        Eigen::Vector2d const direction_dribbler = observer::utility::calculateRotationVector(pos->z());

        double const norm = direction_robot_to_ball.norm() * direction_dribbler.norm();
        if (norm == 0) continue;
        double const angle = std::acos(direction_robot_to_ball.dot(direction_dribbler) / norm);
        // convert angle from radiant to degrees
        double const angle_deg = angle * 180 / L_PI;

        // check if the angle is greater than the angle set in the config
        if (angle_deg > max_angle) {
            setVisionDribblerStatus(identifier, false, time);
            continue;
        }
        setVisionDribblerStatus(identifier, true, time);
    }
}

bool BallFilter::setFilteredVisionData(const std::vector<ssl_interface::SSLBallInfo>& data, const time::TimePoint& time,
                                       size_t camera_id) {
    // Sort ball infos by descending order of confidence
    std::vector<ssl_interface::SSLBallInfo> sorted_data = data;
    std::sort(sorted_data.begin(), sorted_data.end(),
              [](const ssl_interface::SSLBallInfo& a, const ssl_interface::SSLBallInfo& b) {
                  return a.confidence > b.confidence;
              });

    // Iterate over the sorted list and apply filters
    for (const ssl_interface::SSLBallInfo& ball : sorted_data) {
        if (ball.confidence < 0) {
            continue;  // Skip low confidence balls
        }

        // Get ball position
        Eigen::Affine2d ball_position = getBallAffine(ball.position.x(), ball.position.y());

        // Check if the latest ball position and vision data are available
        if (!(getLatestBallPosition().has_value() && getLatestVisionStamp().has_value() &&
              getLatestVisionBallPosition().has_value())) {
            return false;  // Return false if not available
        }

        // Apply filters
        bool accepted = true;
        for (auto& filter : this->filters) {
            if (filter == nullptr) {
                break;
            }

            if (!filter->isActive()) {
                continue;
            }

            if (!filter->computeVisionData(getLatestVisionBallPosition().value(), ball_position,
                                           getLatestVisionStamp().value(), time)) {
                accepted = false;
                break;
            }

            ball_position = filter->getCurrentBallPosition().value();
        }

        if (accepted) {
            // check if the ball is near the dribbler of an enemy robot.
            // If true, set the bll position to the dribbler position of the enemy robot
            checkVisionDribbler(ball_position, time);

            // Update ball velocity and position
            this->latest_ball_velocity =
                calculateBallVelocity(*getLatestVisionBallPosition(), ball_position, *getLatestStamp(), time);
            this->latest_ball_position = ball_position;
            this->latest_vision_ball_position = ball_position;
            this->latest_stamp = time;
            this->latest_vision_stamp = time;
            checkCameraStatus(camera_id, true, time);
            return true;
        }
    }

    checkCameraStatus(camera_id, false, time);

    bool const ball_is_inviable = checkBallInvisible();
    if (ball_is_inviable && !getBallCarryingRobot().has_value()) {
        if (this->world_model == nullptr) {
            return false;
        }
        auto ball_transform = this->world_model->getTransform("ball");
        if (ball_transform.has_value()) {
            auto ball_transform_before = this->world_model->getTransform(
                "ball", "",
                ball_transform->header.stamp - time::Duration(this->world_model->DEFAULT_AVERAGING_INTERVAL));
            if (ball_transform_before.has_value()) {
                double dt = 0.0;
                if ((ball_transform->header.stamp - ball_transform_before->header.stamp).count() != 0) {
                    // NOLINTNEXTLINE(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
                    dt = (time::now() - ball_transform_before->header.stamp).count() /
                         (double)(ball_transform->header.stamp - ball_transform_before->header.stamp).count();
                }
                Eigen::Translation2d translation(
                    ball_transform_before->transform.translation() +
                    (ball_transform->transform.translation() - ball_transform_before->transform.translation()) * dt);

                Eigen::Affine2d last_ball_position = translation * Eigen::Rotation2Dd(0.0);
                this->latest_ball_velocity = calculateBallVelocity(getLatestBallPosition().value(), last_ball_position,
                                                                   getLatestStamp().value(), time);
                checkVisionDribbler(last_ball_position, time);
                this->latest_ball_position = last_ball_position;
                this->latest_stamp = time;
            }
        }
    }

    // No ball was accepted
    return false;
}

bool BallFilter::setInitialVisionValues(const std::vector<ssl_interface::SSLBallInfo>& data,
                                        const time::TimePoint& time) {
    if (data.empty()) {
        return false;
    }
    this->latest_stamp = time;
    double latest_confidence = 0;
    std::optional<ssl_interface::SSLBallInfo> highest_confidence_ball;
    // integrating over the data vector and setting the position with the highest confidence
    for (const ssl_interface::SSLBallInfo& ball : data) {
        if (ball.confidence > latest_confidence) {
            latest_confidence = ball.confidence;
            highest_confidence_ball = ball;
        }
    }
    if (highest_confidence_ball.has_value()) {
        const Eigen::Affine2d best_ball_position =
            getBallAffine(highest_confidence_ball.value().position.x(), highest_confidence_ball.value().position.y());
        this->latest_ball_position = best_ball_position;
        this->latest_vision_ball_position = best_ball_position;
        this->latest_stamp = time;
        this->latest_vision_stamp = time;
        return true;
    }
    return false;
}

transform::TransformWithVelocity BallFilter::getLatestBallTransform() const {
    transform::TransformWithVelocity transform;
    transform.header.stamp = time::now();
    transform.header.child_frame = "ball filter";
    transform.header.parent_frame = "";

    transform.transform = getLatestBallPosition();
    transform.velocity = getLatestBallVelocity();
    return transform;
}

transform::BallInfo BallFilter::getLatestBallInfo() const {
    transform::BallInfo info;
    info.time = time::now();
    if (getRobotWithBall().has_value()) {
        info.state = transform::BallState::IN_ROBOT;
        info.robot = getRobotWithBall().value();
    } else {
        info.state = transform::BallState::ON_FIELD;
        if (!getLatestBallPosition().has_value() || !getLatestBallVelocity().has_value()) {
            return info;
        }
        info.position = std::make_pair(getLatestBallPosition(), getLatestBallVelocity());
    }
    return info;
}
}  // namespace luhsoccer::game_data_provider