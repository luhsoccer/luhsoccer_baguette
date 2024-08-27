#include "time/time.hpp"
#include "utils/utils.hpp"

namespace luhsoccer::time {

Rate::Rate(double freq, std::optional<std::string> name)
    : freq(freq), period_time(1.0 / freq), logger("time::Rate"), name(std::move(name)){};

void Rate::sleep() {
    // sleep for uncleaned periodic time if never executed before
    if (this->last_execute.asSec() == 0.0) {
        std::this_thread::sleep_for(this->period_time);

    } else {
        // measure execution time and sleep for rest of time;
        Duration exec_time = now() - this->last_execute;
        this->latest_execution_times_to_long.push_front(exec_time > this->period_time);
        if (this->latest_execution_times_to_long.size() > NUMBER_OF_SAVED_EXECUTIONS)
            this->latest_execution_times_to_long.pop_back();
        if (exec_time < this->period_time) {
            highPrecisionSleep(this->period_time - exec_time);
        } else {
            int to_long_count = 0;
            for (const bool to_long : this->latest_execution_times_to_long) {
                if (to_long) to_long_count++;
            }
            if (to_long_count >= NUMBER_OF_SAVED_EXECUTIONS * RATIO_OF_TO_LONG_EXECUTIONS_FOR_WARN) {
                if (this->name.has_value()) {
                    this->logger.warning(
                        "Loop '{}' with Rate {:02.2f} took to long. Desired period time is {:.4f}s but took {:.4f}s",
                        this->name.value(), this->freq, 1.0 / this->freq, exec_time.asSec());
                } else {
                    this->logger.warning(
                        "Loop with Rate {:02.2f} took to long. Desired period time is {:.4f}s but took {:.4f}s",
                        this->freq, 1.0 / this->freq, exec_time.asSec());
                }
            }
        }
    }
    this->last_execute = now();
};

LoopStopwatch::LoopStopwatch(const std::string& name, double desired_frequency, size_t window_size)
    : desired_frequency(desired_frequency), window_size(window_size), logger(name){};

void LoopStopwatch::tik() {
    auto time_now = now();
    if (this->last_loop_time) {
        this->loop_times.emplace_back(time_now - last_loop_time.value());
        if (window_size != 0 && this->loop_times.size() > window_size) this->loop_times.pop_front();
    }
    this->last_loop_time = time_now;
}

void LoopStopwatch::printResult() {
    this->logger.info("The measured frequency is {:0.2f}Hz (desired: {:0.2f}Hz) with a window_size of {:d}",
                      this->measuredFrequency(), this->desired_frequency, this->window_size);
}

double LoopStopwatch::measuredFrequency() {
    Duration duration_sum(0.0);
    for (const Duration& d : this->loop_times) {
        duration_sum += d;
    }
    return 1.0 / (duration_sum.asSec() / this->loop_times.size());
}

TimePoint startTime() {
    static TimePoint start = Clock::now();
    return start;
}

}  // namespace luhsoccer::time
