#include "time/calc_time_stopwatch.hpp"
#include <numeric>

namespace luhsoccer::time {

CalcTimeStopwatch::CalcTimeStopwatch(std::string name, size_t window_size)
    : name(std::move(name)), window_size(window_size){};

bool CalcTimeStopwatch::startSection(const std::string& section_name) {
    if (this->started_sections.find(section_name) != this->started_sections.end()) return false;

    this->started_sections[section_name] = now();
    return true;
}

bool CalcTimeStopwatch::endSection(const std::string& section_name) {
    auto start_it = this->started_sections.find(section_name);
    if (start_it == this->started_sections.end()) return false;

    SectionData data{start_it->second, now()};

    auto section_data_it = this->section_data.find(section_name);
    if (section_data_it == this->section_data.end()) {
        this->section_data[section_name] = std::deque<SectionData>();
    }
    if (this->section_data[section_name].size() >= this->window_size) this->section_data[section_name].pop_front();
    this->section_data[section_name].push_back(data);
    this->started_sections.erase(section_name);
    return true;
}

std::optional<Duration> CalcTimeStopwatch::getMeanSectionTime(const std::string& section_name) const {
    auto section_data_it = this->section_data.find(section_name);
    if (section_data_it == this->section_data.end() || section_data_it->second.size() == 0) return std::nullopt;

    Duration sum_duration = std::accumulate(
        section_data_it->second.begin(), section_data_it->second.end(), Duration(0),
        [](const Duration& d, const SectionData& data) { return d + (data.end_time - data.start_time); });

    return sum_duration / section_data_it->second.size();
}

void CalcTimeStopwatch::printSectionTimes() {
    std::vector<std::pair<std::string, Duration>> mean_section_times;
    std::string out = fmt::format("Section times of Stopwatch '{}':", this->name);
    for (auto& [section_name, value] : this->section_data) {
        auto duration = getMeanSectionTime(section_name);
        if (duration.has_value()) {
            mean_section_times.emplace_back(section_name, duration.value());
            out += fmt::format(" '{}': {:0.5f}s", section_name, duration.value().asSec());
        }
    }
    LOG_INFO(this->logger, out);
}

}  // namespace luhsoccer::time