

#pragma once

#include "time/time.hpp"
#include "logger/logger.hpp"

#include <map>
#include <deque>

namespace luhsoccer::time {
class CalcTimeStopwatch {
   public:
    const static size_t DEFAULT_WINDOW_SIZE = 10;
    explicit CalcTimeStopwatch(std::string name, size_t window_size = DEFAULT_WINDOW_SIZE);

    bool startSection(const std::string& name);

    bool endSection(const std::string& name);

    [[nodiscard]] std::optional<Duration> getMeanSectionTime(const std::string& name) const;

    void printSectionTimes();

   private:
    struct SectionData {
        TimePoint start_time;
        TimePoint end_time;
    };
    std::string name;
    size_t window_size;
    std::map<std::string, std::deque<SectionData>> section_data{};
    std::map<std::string, TimePoint> started_sections{};
    logger::Logger logger{"CalcTimeStopwatch"};
};
}  // namespace luhsoccer::time
