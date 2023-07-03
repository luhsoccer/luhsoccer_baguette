#pragma once
#include "spdlog/spdlog.h"
#include "spdlog/fmt/ostr.h"
#include "spdlog/fmt/chrono.h"

constexpr spdlog::source_loc createSourceLoc(const char* filename, int line) { return {filename, line, ""}; }

// Clang-tidy macro warnings are disabled since they're currently the only viable option to get the filename and the
// line. c++20 std::source_location support is still very bad NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define LOG(logger, level, ...) (logger).getImpl()->log(createSourceLoc(__FILE__, __LINE__), level, __VA_ARGS__)

/// stringstream implementation for logger to be used with streams vor convenience
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define LOG_STREAM(logger, level, stream)                \
    std::stringstream ss;                                \
    ss << stream; /*NOLINT(bugprone-macro-parentheses)*/ \
    LOG(logger, level, ss.str());

/**
 * @brief Logs a trace event to every avaliable location
 * Logs a event with level trace using the specified logger. All logger created by this header are multithreading-safe.
 * The trace level should be used in situation when every step of a function should be logged.
 */
#define LOG_TRACE(logger, ...) LOG(logger, spdlog::level::trace, __VA_ARGS__)  // NOLINT(cppcoreguidelines-macro-usage)
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define LOG_TRACE_STREAM(logger, stream) LOG_STREAM(logger, spdlog::level::trace, stream)

/**
 * @brief Logs a debug event to every avaliable location
 * Logs a event with level debug using the specified logger. All logger created by this header are multithreading-safe.
 * The debug level is for information that the user doesn't need but which still can be helpful for debugging.
 */
#define LOG_DEBUG(logger, ...) LOG(logger, spdlog::level::debug, __VA_ARGS__)  // NOLINT(cppcoreguidelines-macro-usage)
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define LOG_DEBUG_STREAM(logger, stream) LOG_STREAM(logger, spdlog::level::debug, stream)

/**
 * @brief Logs a info event to every avaliable location
 * Logs a event with level info using the specified logger. All logger created by this header are multithreading-safe
 * The info level should log general application events that happens expected.
 */
#define LOG_INFO(logger, ...) LOG(logger, spdlog::level::info, __VA_ARGS__)  // NOLINT(cppcoreguidelines-macro-usage)
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define LOG_INFO_STREAM(logger, stream) LOG_STREAM(logger, spdlog::level::info, stream)

/**
 * @brief Logs a warn event to every avaliable location
 * Logs a event with level warning using the specified logger. All logger created by this header are multithreading-safe
 * The warning should log events and applications states that happens unexpected.
 */
#define LOG_WARNING(logger, ...) LOG(logger, spdlog::level::warn, __VA_ARGS__)  // NOLINT(cppcoreguidelines-macro-usage)
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define LOG_WARNING_STREAM(logger, stream) LOG_STREAM(logger, spdlog::level::warn, stream)

/**
 * @brief Logs a erorr event to every avaliable location
 * Logs a event with level error using the specified logger. All logger created by this header are multithreading-safe
 * The error level should only log events that happens unexpected and the application can't be recovered safetly.
 */
#define LOG_ERROR(logger, ...) LOG(logger, spdlog::level::err, __VA_ARGS__)  // NOLINT(cppcoreguidelines-macro-usage)
// NOLINTNEXTLINE(cppcoreguidelines-macro-usage)
#define LOG_ERROR_STREAM(logger, stream) LOG_STREAM(logger, spdlog::level::error, stream)

namespace luhsoccer::logger {

enum class LogColor { WHITE, GREEN, MAGENTA, CYAN, BLUE };

template <typename Mutex>
class GuiSink;

/**
 * @brief Represents a logger
 * Should only be used throught the LOG_ macros.
 */
class Logger {
   private:
    std::shared_ptr<spdlog::logger> logger_impl;

   public:
    /**
     * @brief Construct a new Logger object.
     *
     * @param name The name of the module to log
     * @param color Color to log non-critical messages
     */
    Logger(const std::string& name, const LogColor color = LogColor::WHITE);

    [[nodiscard]] const std::shared_ptr<spdlog::logger>& getImpl() const { return this->logger_impl; }

    static std::shared_ptr<GuiSink<std::mutex>> getGuiSink();
};

// ToDo: Static Logger

}  // namespace luhsoccer::logger