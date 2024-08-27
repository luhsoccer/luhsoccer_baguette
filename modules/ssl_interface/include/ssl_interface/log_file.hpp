#pragma once

#include <vector>
#include <logger/logger.hpp>

namespace google::protobuf {
class Arena;
}

namespace luhsoccer::ssl_interface {

enum class LogFileState { NOT_LOADED, PARSING, RUNNING };

struct LogFileControlHandle {
    std::atomic_bool stop{false};
    std::atomic_size_t current_frame{0};
    std::atomic_size_t max_frames{0};
    std::atomic_int replay_factor{1};
    std::atomic_size_t preload_paths{0};
    std::atomic<LogFileState> state{LogFileState::NOT_LOADED};
};

class SSLInterface;

struct LogPacket;

class LogFile {
   public:
    explicit LogFile(std::string file);
    ~LogFile();
    LogFile(const LogFile& other);
    LogFile(LogFile&& other) noexcept;
    LogFile& operator=(const LogFile& other);
    LogFile& operator=(LogFile&& other) noexcept;

    void load();

    void replay(SSLInterface& interface, LogFileControlHandle& control_handle);

    [[nodiscard]] size_t size() const;

   private:
    void replayImpl(SSLInterface& interface, LogFileControlHandle& control_handle);

    template <typename P, typename M>
    void addPacket(const char* message, int size, std::int64_t timestamp, M converter, google::protobuf::Arena& arena);

   private:
    std::string file;
    constexpr static int32_t SUPPORTED_FILE_VERSION = 1;
    std::vector<LogPacket> packets;
    logger::Logger logger{"log_file"};
};

}  // namespace luhsoccer::ssl_interface