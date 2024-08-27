/**
 * @file time.h
 * @author Fabrice Zeug (zeug@stud.uni-hannover.de)
 * @brief Basic time and rate operations
 * @version 0.1
 * @date 2022-08-04
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <chrono>
#include <cmath>
#include <optional>
#include <deque>

#include "logger/logger.hpp"

namespace luhsoccer::time {

constexpr double SEC_TO_NSEC_MULTIPLIER = 1e9;
constexpr long SEC_TO_NSEC_MULTIPLIER_INT = SEC_TO_NSEC_MULTIPLIER;
/**
 * @brief Default time duration object
 *
 */
class Duration : public std::chrono::nanoseconds {
   public:
    // inherit all conversion etc. from chrono nanoseconds
    using std::chrono::nanoseconds::nanoseconds;
    constexpr inline Duration(const std::chrono::nanoseconds& d) : std::chrono::nanoseconds(d){};
    // operator std::chrono::nanoseconds() const {
    //   return std::chrono::nanoseconds(count());
    // };

    /**
     * @brief Construct a new Duration object from seconds given as double
     *
     * @param secs duration in seconds
     */
    constexpr inline Duration(double secs) : std::chrono::nanoseconds((int64_t)(secs * SEC_TO_NSEC_MULTIPLIER)){};

    /**
     * @brief Construct a new Duration object from seconds and nanoseconds given
     * as int64_t
     *
     * @param secs duration in seconds
     * @param nsecs duration in nanoseconds
     */
    constexpr inline Duration(int64_t secs, int64_t nsecs)
        : std::chrono::nanoseconds((int64_t)(secs * SEC_TO_NSEC_MULTIPLIER_INT) + nsecs){};

    /**
     * @brief Duration as seconds
     */
    // double is large enough for common durations
    // NOLINTNEXTLINE(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
    [[nodiscard]] inline double asSec() const { return count() / SEC_TO_NSEC_MULTIPLIER; };
    /**
     * @brief Duration as nanoseconds
     */
    [[nodiscard]] inline int64_t asNSec() const { return count(); }
};

/**
 * @brief Point in time
 *
 */

using Clock = std::chrono::steady_clock;

class TimePoint : public std::chrono::time_point<Clock> {
   public:
    // inherit all conversion etc. from chrono time_point
    using std::chrono::time_point<Clock>::time_point;
    inline TimePoint(const std::chrono::time_point<Clock>& time) : std::chrono::time_point<Clock>(time){};

    inline TimePoint(double time_in_sec) : TimePoint(Duration(time_in_sec)){};

    /**
     * @brief TimePoint as seconds
     */
    // double is large enough for common durations
    // NOLINTNEXTLINE(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
    [[nodiscard]] inline double asSec() const { return time_since_epoch().count() / SEC_TO_NSEC_MULTIPLIER; };
    /**
     * @brief TimePoint as nanoseconds
     */
    [[nodiscard]] inline int64_t asNSec() const { return time_since_epoch().count(); }
};

TimePoint startTime();

/**
 * @brief The time that has passed since the program was started
 */
inline Duration timeSinceStart() { return Clock::now() - startTime(); }

/**
 * @brief The current point in time in steady clock
 */
inline TimePoint now() { return TimePoint(0) + (Clock::now() - startTime()); }

/**
 * @brief Provides a sleeping function that accounts for execute times, intended
 * to be used in loops.
 *
 */
class Rate {
   public:
    /**
     * @brief Construct a new Rate object
     *
     * @param freq desired frequency of the loop
     */
    Rate(double freq, std::optional<std::string> name = std::nullopt);

    /**
     * @brief sleeps to sleep for desired rate
     *
     */
    void sleep();

   private:
    // Logger logger_;
    /**
     * @brief desired frequency
     */
    double freq;

    /**
     * @brief time before last execution
     *
     */
    TimePoint last_execute;
    /**
     * @brief desired period_time = 1/f
     *
     */
    Duration period_time;

    /**
     * @brief logger for prints
     *
     */
    logger::Logger logger;

    /**
     * @brief name to identify the loop in warnings
     *
     */
    std::optional<std::string> name;

    constexpr static int NUMBER_OF_SAVED_EXECUTIONS = 10;
    constexpr static double RATIO_OF_TO_LONG_EXECUTIONS_FOR_WARN = 0.5;
    /**
     * @brief latest
     *
     */
    std::deque<bool> latest_execution_times_to_long;
};

class LoopStopwatch {
   public:
    LoopStopwatch(const std::string& name = "time::LoopStopwatch", double desired_frequency = 0.0,
                  size_t window_size = 0);

    void tik();

    double measuredFrequency();

    void printResult();

   private:
    std::optional<TimePoint> last_loop_time;
    std::deque<Duration> loop_times;
    double desired_frequency;
    size_t window_size;
    logger::Logger logger;
};

}  // namespace luhsoccer::time

namespace std {

/**
 * @brief Convert Duration to string for printing
 */
// NOLINTNEXTLINE(readability-identifier-naming) - std function naming style
inline std::string to_string(const luhsoccer::time::Duration& d) {
    long nanos = d.count() % luhsoccer::time::SEC_TO_NSEC_MULTIPLIER_INT;
    constexpr int DECIMAL_PLACES = 8;
    int precision = DECIMAL_PLACES;
    if (nanos != 0) {
        precision = DECIMAL_PLACES - std::min(DECIMAL_PLACES, (int)std::log10(std::abs(nanos)));
    }
    // double is large enough for common durations
    // NOLINTNEXTLINE(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
    return std::to_string((int)trunc(d.count() / luhsoccer::time::SEC_TO_NSEC_MULTIPLIER)) + "." +
           std::string(precision, '0') + std::to_string(std::abs(nanos));
}

/**
 * @brief Convert TimePoint to string for printing
 */
// NOLINTNEXTLINE(readability-identifier-naming) - std function naming style
inline std::string to_string(const luhsoccer::time::TimePoint& time) { return to_string(time.time_since_epoch()); }

}  // namespace std

namespace luhsoccer::time {

inline auto format_as(const luhsoccer::time::Duration& time) { return std::to_string(time); }
inline auto format_as(const luhsoccer::time::TimePoint& time) { return std::to_string(time); }
};  // namespace luhsoccer::time