#pragma once

#include <vector>
#include <variant>
#include <optional>
#include <climits>
#include <logger/logger.hpp>
#include "ssl_interface/ssl_interface.hpp"

namespace luhsoccer::ssl_interface {

class LogFile {
   public:
    LogFile(std::string file);

    void replayAtMaxSpeed(SSLInterface& interface);

    void load();

    void seekBy(int offset);

   private:
    std::string file;
    template <typename P, typename M>
    void addPacket(const char* message, int size, std::function<M(P)> converter) {
        P packet;
        if (!packet.ParseFromArray(message, size)) {
            LOG_WARNING(logger, "Error while try to parse packet");
        } else {
            packets.emplace_back(converter(packet));
        }
    }
    constexpr static int32_t SUPPORTED_FILE_VERSION = 1;
    std::vector<std::variant<SSLWrapperData, SSLGameControllerData>> packets;
    size_t current_pos{0};
    logger::Logger logger{"log_file", logger::LogColor::GREEN};
};

}  // namespace luhsoccer::ssl_interface