
#include "vision_processor/vision_processor.hpp"
#include "vision_processor/vision_processor_events.hpp"
#include "event_system/event_system.hpp"
#include "ssl_interface/ssl_types.hpp"

namespace luhsoccer::vision_processor {
// NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index) - suppress using id as array index
void VisionProcessor::setup([[maybe_unused]] event_system::EventSystem& event_system) {
    event_system.registerEventHandler<ssl_interface::NewVisionDataEvent>(
        [this](event_system::EventContext<ssl_interface::NewVisionDataEvent> ctx) {
            static std::mutex mutex;

            mutex.lock();

            this->last_cam_data[ctx.event.data.camera_id] = std::make_pair(ctx.event.data, time::now());

            invalidateVisionData();

            invalidateLatestTruth();

            this->checkVisionDataPlausability(ctx.event.data.camera_id);

            ProcessedVisionData data = this->mergeVisionData(ctx.event.data.camera_id);

            this->latest_truth = data;

            mutex.unlock();

            ctx.system.fireEvent(NewProcessedVisionDataEvent(data));
        },
        true);

    event_system.registerEventHandler<TeleportEvent>(
        [this](event_system::EventContext<TeleportEvent> ctx) { this->handleTeleport(ctx.event.data); });
}

void VisionProcessor::checkVisionDataPlausability(size_t cam_index) {
    constexpr double MAX_ROBOT_VEL = 4.0;
    constexpr double MAX_BALL_VEL = 6.5;
    constexpr double VISION_TOLERANCE = 0.1;

    if (!this->latest_truth.has_value()) return;

    auto& [vision_data, _] = this->last_cam_data[cam_index];
    // Compute delta between frames
    auto delta_s = vision_data.timestamp_capture.asSec() - this->latest_truth->time_captured.asSec();

    // Filter out all balls too far away from the latest truth
    if (!this->wasBallRecentlyTeleported(time::now())) {
        if (this->latest_truth->ball.has_value()) {
            auto latest_ball_pos = *this->latest_truth->ball;
            for (auto it = vision_data.balls.begin(); it != vision_data.balls.end();) {
                auto dist = (latest_ball_pos.translation() - it->position.head<2>()).norm();
                if (dist > std::max(MAX_BALL_VEL * delta_s, VISION_TOLERANCE)) {
                    it = vision_data.balls.erase(it);
                } else {
                    ++it;
                }
            }
        }
    }

    // Filter out all robots too far away from the latest truth
    // Blue
    {
        for (auto it = vision_data.blue_robots.begin(); it != vision_data.blue_robots.end();) {
            if (it->id < MAX_ROBOTS_PER_TEAM) {
                auto latest_robot_pos = this->latest_truth->blue_robots[it->id];
                if (latest_robot_pos.has_value()) {
                    if (!this->wasRecentlyTeleported(it->id, TeamColor::BLUE, time::now())) {
                        auto dist = (latest_robot_pos->translation() - it->transform.translation()).norm();
                        if (dist > std::max(MAX_ROBOT_VEL * delta_s, VISION_TOLERANCE)) {
                            it = vision_data.blue_robots.erase(it);
                            continue;
                        }
                    }
                }
            }

            ++it;
        }
    }

    // Yellow
    {
        for (auto it = vision_data.yellow_robots.begin(); it != vision_data.yellow_robots.end();) {
            if (it->id < MAX_ROBOTS_PER_TEAM) {
                auto latest_robot_pos = this->latest_truth->yellow_robots[it->id];
                if (!this->wasRecentlyTeleported(it->id, TeamColor::YELLOW, time::now())) {
                    if (latest_robot_pos.has_value()) {
                        auto dist = (latest_robot_pos->translation() - it->transform.translation()).norm();
                        if (dist > std::max(MAX_ROBOT_VEL * delta_s, VISION_TOLERANCE)) {
                            it = vision_data.yellow_robots.erase(it);
                            continue;
                        }
                    }
                }
            }

            ++it;
        }
    }

    this->clearRecentTeleports(time::now());
}

void VisionProcessor::invalidateVisionData() {
    const auto time = time::now();

    for (auto it = this->last_cam_data.begin(); it != this->last_cam_data.end();) {
        const auto delta = time - it->second.second;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(delta) > DATA_MAX_AGE) {
            it = last_cam_data.erase(it);
        } else {
            ++it;
        }
    }
}

void VisionProcessor::invalidateLatestTruth() {
    if (!this->latest_truth.has_value()) return;

    const auto delta = time::now() - this->latest_truth->time_captured;

    if (std::chrono::duration_cast<std::chrono::milliseconds>(delta) > DATA_MAX_AGE) {
        this->latest_truth = std::nullopt;
    }
}

ProcessedVisionData VisionProcessor::mergeVisionData(size_t camera_id) {
    // Merge all stored data

    ProcessedVisionData processed_data;
    processed_data.camera_id = camera_id;
    processed_data.time_captured = this->last_cam_data[camera_id].first.timestamp_capture;
    processed_data.time_sent = this->last_cam_data[camera_id].first.timestamp_sent;

    Eigen::Vector3d ball_avg{Eigen::Vector3d::Zero()};
    size_t ball_count = 0;

    std::array<size_t, MAX_ROBOTS_PER_TEAM> yellow_count{};
    std::array<size_t, MAX_ROBOTS_PER_TEAM> blue_count{};

    auto add_avg = [](std::optional<Eigen::Affine2d>& val1, const Eigen::Affine2d& val2, size_t& count) {
        if (!val1.has_value()) {
            count = 1;
            val1.emplace(val2);
        } else {
            count++;
            val1.value().matrix() += val2.matrix();
            // val2.affine().addTo(val1.value());
        }
    };

    for (const auto& [cam_id, info] : last_cam_data) {
        const auto& [data, _] = info;

        for (const auto& ball : data.balls) {
            ball_avg += ball.position;
            ball_count++;
        }

        for (const auto& robot : data.blue_robots) {
            add_avg(processed_data.blue_robots[robot.id], robot.transform, blue_count[robot.id]);
        }

        for (const auto& robot : data.yellow_robots) {
            add_avg(processed_data.yellow_robots[robot.id], robot.transform, yellow_count[robot.id]);
        }
    }

    for (size_t id = 0; id < processed_data.blue_robots.size(); id++) {
        if (!processed_data.blue_robots[id].has_value()) continue;
        processed_data.blue_robots[id]->matrix() *= 1.0 / static_cast<double>(blue_count[id]);
    }

    for (size_t id = 0; id < processed_data.yellow_robots.size(); id++) {
        if (!processed_data.yellow_robots[id].has_value()) continue;
        processed_data.yellow_robots[id]->matrix() *= 1.0 / static_cast<double>(yellow_count[id]);
    }

    if (ball_count == 0) {
        processed_data.ball = std::nullopt;
    } else {
        ball_avg *= 1.0 / static_cast<double>(ball_count);
        processed_data.ball = Eigen::Translation2d(ball_avg.head<2>()) * Eigen::Rotation2Dd(0);
    }

    return processed_data;
}

void VisionProcessor::handleTeleport(const TeleportData& data) {
    const std::unique_lock lock(this->teleport_mtx);
    this->recent_teleports.push_back(data);
}

bool VisionProcessor::wasRecentlyTeleported(size_t id, TeamColor team, time::TimePoint time) {
    const std::unique_lock lock(this->teleport_mtx);
    for (const auto& teleport : this->recent_teleports) {
        if (std::holds_alternative<TeleportData::RobotTeleport>(teleport.teleport_type)) {
            const auto& robot_teleport = std::get<TeleportData::RobotTeleport>(teleport.teleport_type);
            if (robot_teleport.id == id && robot_teleport.team == team) {
                if ((time - teleport.time) <= this->TELEPORT_TIMEOUT) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool VisionProcessor::wasBallRecentlyTeleported(time::TimePoint time) {
    const std::unique_lock lock(this->teleport_mtx);
    for (const auto& teleport : this->recent_teleports) {
        if (std::holds_alternative<TeleportData::BallTeleport>(teleport.teleport_type)) {
            if ((time - teleport.time) <= this->TELEPORT_TIMEOUT) {
                return true;
            }
        }
    }
    return false;
}

void VisionProcessor::clearRecentTeleports(time::TimePoint time) {
    const std::unique_lock lock(this->teleport_mtx);
    std::erase_if(this->recent_teleports, [&time, this](const TeleportData& teleport) {
        return (time - teleport.time) > this->TELEPORT_TIMEOUT;
    });
}
// NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
}  // namespace luhsoccer::vision_processor