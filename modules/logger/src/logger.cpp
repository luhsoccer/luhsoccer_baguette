#include "logger/logger.hpp"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "logger/gui_sink.hpp"
#include "spdlog/sinks/rotating_file_sink.h"
#include "utils/utils.hpp"

namespace luhsoccer::logger {

// std::shared_ptr<GuiSinkMt> Logger::gui_sink = std::make_shared<GuiSinkMt>();

spdlog::level::level_enum getLogLevelFromEnv() {
    const char* env_log_level_cstr = std::getenv("LOG_LEVEL");

    if (!env_log_level_cstr) return spdlog::level::info;
    std::string env_log_level = env_log_level_cstr;
    if (env_log_level == "TRACE") {
        return spdlog::level::trace;
    } else if (env_log_level == "DEBUG") {
        return spdlog::level::debug;

    } else if (env_log_level == "WARNING") {
        return spdlog::level::warn;

    } else if (env_log_level == "ERROR") {
        return spdlog::level::err;

    } else if (env_log_level == "CRITICAL") {
        return spdlog::level::critical;

    } else if (env_log_level == "OFF") {
        return spdlog::level::off;
    } else {
        return spdlog::level::info;
    }
}

Logger::Logger(const std::string& name, const LogColor color) {
    static std::mutex registry_mutex;
    std::lock_guard lock(registry_mutex);
    auto logger_ptr = spdlog::get(name);

    if (logger_ptr != nullptr) {  // Use existing logger if exists
        this->logger_impl = std::move(logger_ptr);
    } else {
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
#ifndef _WIN32
        auto logger_color = console_sink->white;

        switch (color) {
            case LogColor::GREEN:
                logger_color = console_sink->green;
                break;
            case LogColor::MAGENTA:
                logger_color = console_sink->magenta;
                break;
            case LogColor::CYAN:
                logger_color = console_sink->cyan;
                break;
            case LogColor::BLUE:
                logger_color = console_sink->blue;
                break;
            default:
                logger_color = console_sink->white;
                break;
        }

        // can we change so that errors are red, warnings yellow, and infos white by default?
        console_sink->set_color(spdlog::level::info, logger_color);
        console_sink->set_color(spdlog::level::debug, logger_color);
        console_sink->set_color(spdlog::level::trace, logger_color);
#else
        auto logger_color = 0b111;

        switch (color) {
            case LogColor::GREEN:
                logger_color = 0b010;
                break;
            case LogColor::MAGENTA:
                logger_color = 0b110;
                break;
            case LogColor::CYAN:
                logger_color = 0b1001;
                break;
            case LogColor::BLUE:
                logger_color = 0b001;
                break;
            default:
                logger_color = 0b111;
                break;
        }

        console_sink->set_color(spdlog::level::info, logger_color);
        console_sink->set_color(spdlog::level::debug, logger_color);
        console_sink->set_color(spdlog::level::trace, logger_color);
#endif
#ifdef BAGUETTE_LOCAL_MODE                              // @todo log also in trace in headless / deploy mode
        console_sink->set_level(getLogLevelFromEnv());  // Log everything to the console in dev mode
#else
        console_sink->set_level(spdlog::level::warn);  // Log only warnings to the console in deploy mode
#endif
        auto gui_sink = getGuiSink();

        std::optional<spdlog::logger> logger;

        // Gated behind flag because of: https://github.com/gabime/spdlog/issues/937
        if (std::getenv("BAGUETTE_ENABLE_FILE_LOGGING")) {
            constexpr size_t MAX_FILES = 20;
            constexpr size_t MAX_FILE_SIZE = 1048576 * 5;

            static std::string logger_path = getBaguetteDirectory().append("logs/baguette.log").string();

            static std::shared_ptr<spdlog::sinks::rotating_file_sink<std::mutex>> file_sink =
                std::make_shared<spdlog::sinks::rotating_file_sink<std::mutex>>(logger_path, MAX_FILE_SIZE, MAX_FILES,
                                                                                true);

            logger = spdlog::logger(name, {console_sink, gui_sink, file_sink});
        } else {
            logger = spdlog::logger(name, {console_sink, gui_sink});
        }

        if (!logger.has_value()) {
            // Should never happen
            throw std::runtime_error("Failed to create logger");
        }

        logger->set_level(spdlog::level::trace);
        logger->set_pattern("%^[%L/%H:%M:%S:%e][%n][%s:%#] %v %$");
        logger->flush_on(spdlog::level::err);
        this->logger_impl = std::make_shared<spdlog::logger>(logger.value());
        spdlog::register_logger(this->logger_impl);
    }
}

std::shared_ptr<GuiSinkMt> Logger::getGuiSink() {
    static std::shared_ptr<GuiSinkMt> sink = std::make_shared<GuiSinkMt>();

    return sink;
}

}  // namespace luhsoccer::logger