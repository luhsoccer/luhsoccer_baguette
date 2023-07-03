#pragma once

#include "spdlog/sinks/base_sink.h"
#include <iostream>
#include "spdlog/details/null_mutex.h"
#include <mutex>
#include <rigtorp/MPMCQueue.h>
#include <ctime>
#include <utility>

namespace luhsoccer::logger {

struct LoggerDetails {
    // std::string lvlPrefix;

    LoggerDetails() = default;

    LoggerDetails(std::string payload, spdlog::level::level_enum lvl, std::string logger_name, std::string location,
                  spdlog::log_clock::time_point time) noexcept
        : payload(std::move(payload)),
          lvl(lvl),
          logger_name(std::move(logger_name)),
          location(std::move(location)),
          time(time){};
    std::string payload;
    spdlog::level::level_enum lvl{spdlog::level::off};
    std::string logger_name;
    std::string location;
    spdlog::log_clock::time_point time;
};

template <typename Mutex>
class GuiSink : public spdlog::sinks::base_sink<Mutex> {
   private:
    rigtorp::MPMCQueue<LoggerDetails> queue{128};

   protected:
    void sink_it_(const spdlog::details::log_msg& msg) override {
        // char buffer[5];
        // sprintf(buffer, "[%02d]", msg.level);
        std::string payload(msg.payload.data(), msg.payload.size());
        std::string logger_name(msg.logger_name.data(), msg.logger_name.size());

        std::string full_filename = std::string(msg.source.filename);
        auto begin = full_filename.begin();
        auto filename_start = full_filename.end() - 1;
        auto filename_end = full_filename.end();

        while (*filename_start != '/' && *filename_start != '\\') {
            if (filename_start != begin) {
                filename_start--;
            } else {
                break;
            }
        }

        std::string formatted_location(std::string(filename_start + 1, filename_end) + ":" +
                                       std::to_string(msg.source.line));

        if (!queue.try_emplace(std::move(payload), msg.level, std::move(logger_name), std::move(formatted_location),
                               msg.time)) {
            // TODO better way?
            // std::cerr << "Logger queue full!" << std::endl;
        }
    }

    void flush_() override {
        // empty
    }

   public:
    rigtorp::MPMCQueue<LoggerDetails>& getQueue() { return this->queue; }
};

using GuiSinkMt = GuiSink<std::mutex>;
using GuiSinkSt = GuiSink<spdlog::details::null_mutex>;

}  // namespace luhsoccer::logger
