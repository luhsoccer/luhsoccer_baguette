#include <cstddef>

#include "google/protobuf/arena.h"

#include "ssl_interface/log_file.hpp"
#include "log_file/log_file_parser.hpp"
#include "ssl_vision_wrapper.pb.h"
#include "ssl_gc_referee_message.pb.h"
#include "ssl_types_converter/ssl_types_converter.hpp"
#include "ssl_interface/ssl_interface.hpp"

#include "utils/utils.hpp"
#include "fmt/ranges.h"

namespace luhsoccer::ssl_interface {
struct LogPacket {
    time::TimePoint timestamp;
    std::variant<SSLWrapperData, SSLGameControllerData> data;
};

LogFile::LogFile(std::string file) : file(std::move(file)) {}

LogFile::~LogFile() = default;
LogFile::LogFile(const LogFile& other) = default;
LogFile::LogFile(LogFile&& other) noexcept = default;
LogFile& LogFile::operator=(const LogFile& other) = default;
LogFile& LogFile::operator=(LogFile&& other) noexcept = default;

template <typename P, typename M>
void LogFile::addPacket(const char* message, int size, std::int64_t timestamp, M converter,
                        google::protobuf::Arena& arena) {
    P* packet = google::protobuf::Arena::CreateMessage<P>(&arena);
    if (!packet->ParseFromArray(message, size)) {
        logger.warning("Error while try to parse packet");
    } else {
        packets.emplace_back(time::TimePoint(std::chrono::nanoseconds(timestamp)), converter(*packet));
    }
}

void LogFile::load() {
    packets.clear();

    LogFileParser parser{file};
    parser.readHeader();

    google::protobuf::ArenaOptions options;
    options.start_block_size = 2 << 8;
    options.max_block_size = 2 << 16;
    google::protobuf::Arena arena(options);

    while (true) {
        const auto message = parser.readMessage();
        if (!message) [[unlikely]] {
            break;
        }

        if (arena.SpaceAllocated() >= (2 << 22)) [[unlikely]] {
            arena.Reset();
        }

        switch (message->header.message_type) {
            case LogFileParser::MESSAGE_SSL_REFBOX_2013:
                addPacket<Referee>(message->message, message->header.message_size, message->header.timestamp,
                                   converter::parseRefereeData, arena);
                break;
            case LogFileParser::MESSAGE_SSL_VISION_2014:
                addPacket<proto::ssl_vision::SSL_WrapperPacket>(message->message, message->header.message_size,
                                                                message->header.timestamp, converter::parseWrapperData,
                                                                arena);
                break;
            default:
                // Ignore all other messages by default
                break;
        }
    }
}

size_t LogFile::size() const { return this->packets.size(); }

void LogFile::replayImpl(SSLInterface& interface, LogFileControlHandle& control_handle) {
    control_handle.state.store(LogFileState::PARSING);
    this->load();
    control_handle.state.store(LogFileState::RUNNING);
    control_handle.max_frames = packets.size();
    control_handle.current_frame = 0;
    time::TimePoint start_replay = time::now();
    auto start_timestamp = packets[0].timestamp;
    while (control_handle.current_frame < packets.size()) {
        auto& packet = packets[control_handle.current_frame];

        auto time_point = packet.timestamp;

        auto game_log_time = time_point - start_timestamp;
        if (time_point < start_timestamp) {
            // Going backwards through the log
            game_log_time = start_timestamp - time_point;
        }

        start_timestamp = time_point;

        auto real_time_diff = time::now() - start_replay;
        start_replay = time::now();

        auto replay_factor_abs = std::abs(control_handle.replay_factor);

        if (replay_factor_abs != 0) {
            if (game_log_time > real_time_diff) {
                auto sleep_time = (game_log_time / replay_factor_abs) - (real_time_diff / replay_factor_abs);
                if (sleep_time < std::chrono::milliseconds(1)) {
                    continue;
                }
                highPrecisionSleep(std::chrono::duration_cast<std::chrono::milliseconds>(sleep_time));
            }
        } else {
            // Sleep when the log replay is paused
            highPrecisionSleep(std::chrono::milliseconds(100));
        }

        size_t start_frame = control_handle.current_frame;
        marker::LineStrip line_strip("", "log_ball");
        std::vector<marker::Point> ball_points;
        std::unordered_map<unsigned int, std::vector<marker::Point>> robot_points;
        std::size_t preload = control_handle.preload_paths;
        for (size_t i = start_frame; i < std::min(start_frame + preload, packets.size()); i++) {
            if (std::holds_alternative<SSLWrapperData>(packets[i].data)) {
                auto& data = std::get<SSLWrapperData>(packets[i].data);
                if (data.vision) {
                    if (!data.vision->balls.empty()) {
                        ball_points.emplace_back(data.vision->balls[0].position.x(),
                                                 data.vision->balls[0].position.y());
                    }
                    for (const auto& robot : data.vision->blue_robots) {
                        robot_points[robot.id].emplace_back(robot.transform.translation().x(),
                                                            robot.transform.translation().y());
                    }
                    for (const auto& robot : data.vision->yellow_robots) {
                        robot_points[robot.id + 100].emplace_back(robot.transform.translation().x(),
                                                                  robot.transform.translation().y());
                    }
                }
            }
        }

        for (const auto& points : robot_points) {
            marker::LineStrip robot_strip("", "log_robot", points.first);
            if (points.first < 100) {
                robot_strip.setColor(marker::Color::BLUE());
            } else {
                robot_strip.setColor(marker::Color::YELLOW());
            }
            robot_strip.setPoints(points.second);
            interface.ms.displayMarker(robot_strip);
        }

        line_strip.setPoints(std::move(ball_points));
        line_strip.setColor(marker::Color::ORANGE());
        interface.ms.displayMarker(line_strip);

        if (std::holds_alternative<SSLWrapperData>(packet.data)) {
            interface.processWrapperData<VisionDataSource::GAME_LOG>(std::get<SSLWrapperData>(packet.data));
        } else if (std::holds_alternative<SSLGameControllerData>(packet.data)) {
            interface.processGameControllerData<GameControllerDataSource::GAME_LOG>(
                std::get<SSLGameControllerData>(packet.data));
        }

        int step_size = 1 * control_handle.replay_factor;

        if (step_size < 0 && control_handle.current_frame < static_cast<size_t>(-step_size)) {
            logger.info("Reached beginning of log file");
            break;
        }

        control_handle.current_frame.fetch_add(step_size);
    }

    logger.info("Reached end of replay");
    control_handle.state.store(LogFileState::NOT_LOADED);
}

void LogFile::replay(SSLInterface& interface, LogFileControlHandle& control_handle) {
    std::thread t([&, this] { this->replayImpl(interface, control_handle); });
    t.detach();
}
}  // namespace luhsoccer::ssl_interface
