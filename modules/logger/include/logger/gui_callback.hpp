#pragma once

#include <logger/logger.hpp>
#include <functional>

namespace luhsoccer::logger {

struct LoggerDetails {
    LoggerDetails() = default;

    LoggerDetails(std::string payload, logger::LogLevel lvl, std::string logger_name, std::string location,
                  std::chrono::system_clock::time_point time) noexcept
        : payload(std::move(payload)),
          lvl(lvl),
          logger_name(std::move(logger_name)),
          location(std::move(location)),
          time(time){};
    std::string payload;
    logger::LogLevel lvl{LogLevel::OFF};
    std::string logger_name;
    std::string location;
    std::chrono::system_clock::time_point time;
};

void readLogMessages(std::function<void(LoggerDetails)>);

}  // namespace luhsoccer::logger
