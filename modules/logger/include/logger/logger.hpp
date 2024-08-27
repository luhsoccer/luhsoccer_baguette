#pragma once

#include <fmt/core.h>
#include <fmt/format.h>
#include <source_location>
#include <utility>
#include <atomic>

namespace std {
class mutex;
}

namespace spdlog {
class logger;
}

namespace luhsoccer::logger {

class CustomSourceLog {
   public:
    CustomSourceLog(std::string filename, int line) : filename(std::move(filename)), line(line) {}

    std::string filename;
    int line;
};

enum class LogColor { WHITE, GREEN, MAGENTA, CYAN, BLUE };

enum class LogLevel { TRACE, DEBUG, INFO, WARNING, ERROR, OFF };

template <typename Mutex>
class GuiSink;

template <typename Char, typename... Args>
class BasicFormatString {
   private:
    fmt::basic_string_view<Char> str;

   public:
    std::source_location loc;

    template <typename S, FMT_ENABLE_IF(std::is_convertible<const S&, fmt::basic_string_view<Char>>::value)>
    consteval FMT_INLINE BasicFormatString(const S& s, std::source_location loc = std::source_location::current())
        : str(s), loc(loc) {
        static_assert(fmt::detail::count<(std::is_base_of<fmt::detail::view, fmt::remove_reference_t<Args>>::value &&
                                          std::is_reference<Args>::value)...>() == 0,
                      "passing views as lvalues is disallowed");
        if constexpr (fmt::detail::count_named_args<Args...>() == fmt::detail::count_statically_named_args<Args...>()) {
            using Checker = fmt::detail::format_string_checker<Char, fmt::remove_cvref_t<Args>...>;
            fmt::detail::parse_format_string<true>(str, Checker(s));
        }
    }
    BasicFormatString(fmt::runtime_format_string<Char> fmt) : str(fmt.str) {}

    FMT_INLINE operator fmt::basic_string_view<Char>() const { return str; }
    FMT_INLINE auto get() const -> fmt::basic_string_view<Char> { return str; }
};

template <typename... Args>
using FormatString = BasicFormatString<char, fmt::type_identity_t<Args>...>;

/**
 * @brief Represents a logger
 * Should only be used throught the LOG_ macros.
 */
class Logger {
   private:
    std::shared_ptr<spdlog::logger> logger_impl;
    std::atomic_bool active{true};

   public:
    ~Logger();
    Logger(const Logger&);
    Logger(Logger&&) noexcept;
    Logger& operator=(const Logger&);
    Logger& operator=(Logger&&) noexcept;
    /**
     * @brief Construct a new Logger object.
     *
     * @param name The name of the module to log
     * @param color Color to log non-critical messages
     */
    Logger(const std::string& name, const LogColor color = LogColor::WHITE);

    static std::shared_ptr<GuiSink<std::mutex>> getGuiSink();

    void setActive(bool active) { this->active = active; }

    void trace(std::string_view message, std::source_location loc = std::source_location::current()) const {
        this->logImpl(loc, LogLevel::TRACE, message);
    }

    template <typename... T>
    void trace(FormatString<T...> fmt, T&&... args) const {
        this->logImpl(fmt.loc, LogLevel::TRACE, fmt::vformat(fmt, fmt::make_format_args(args...)));
    }

    void debug(std::string_view message, std::source_location loc = std::source_location::current()) const {
        this->logImpl(loc, LogLevel::DEBUG, message);
    }

    template <typename... T>
    void debug(FormatString<T...> fmt, T&&... args) const {
        this->logImpl(fmt.loc, LogLevel::DEBUG, fmt::vformat(fmt, fmt::make_format_args(args...)));
    }

    void info(std::string_view message, std::source_location loc = std::source_location::current()) const {
        this->logImpl(loc, LogLevel::INFO, message);
    }

    template <typename... T>
    void info(FormatString<T...> fmt, T&&... args) const {
        this->logImpl(fmt.loc, LogLevel::INFO, fmt::vformat(fmt, fmt::make_format_args(args...)));
    }

    void warning(std::string_view message, std::source_location loc = std::source_location::current()) const {
        this->logImpl(loc, LogLevel::WARNING, message);
    }

    template <typename... T>
    void warning(FormatString<T...> fmt, T&&... args) const {
        this->logImpl(fmt.loc, LogLevel::WARNING, fmt::vformat(fmt, fmt::make_format_args(args...)));
    }

    void error(std::string_view message, std::source_location loc = std::source_location::current()) const {
        this->logImpl(loc, LogLevel::ERROR, message);
    }

    template <typename... T>
    void error(FormatString<T...> fmt, T&&... args) const {
        this->logImpl(fmt.loc, LogLevel::ERROR, fmt::vformat(fmt, fmt::make_format_args(args...)));
    }

    void logImpl(const std::source_location& loc, LogLevel level, std::string_view message) const;
    void logImpl(const CustomSourceLog& loc, LogLevel level, std::string_view message) const;
};

// ToDo: Static Logger

}  // namespace luhsoccer::logger