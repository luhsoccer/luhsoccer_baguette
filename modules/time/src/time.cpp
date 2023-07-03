#include "time/time.hpp"

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
            std::this_thread::sleep_for(this->period_time - exec_time);
        } else {
            int to_long_count = 0;
            for (const bool to_long : this->latest_execution_times_to_long) {
                if (to_long) to_long_count++;
            }
            if (to_long_count >= NUMBER_OF_SAVED_EXECUTIONS * RATIO_OF_TO_LONG_EXECUTIONS_FOR_WARN) {
                if (this->name.has_value()) {
                    LOG_WARNING(
                        this->logger,
                        "Loop '{}' with Rate {:02.2f} took to long. Desired period time is {:.4f}s but took {:.4f}s",
                        this->name.value(), this->freq, 1.0 / this->freq, exec_time.asSec());
                } else {
                    LOG_WARNING(this->logger,
                                "Loop with Rate {:02.2f} took to long. Desired period time is {:.4f}s but took {:.4f}s",
                                this->freq, 1.0 / this->freq, exec_time.asSec());
                }
            }
        }
    }
    this->last_execute = now();
};
}  // namespace luhsoccer::time
