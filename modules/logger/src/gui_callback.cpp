#include "logger/gui_callback.hpp"
#include <gui_sink.hpp>

namespace luhsoccer::logger {

void readLogMessages(std::function<void(LoggerDetails)> callback) {
    auto gui_sink = Logger::getGuiSink();

    while (!gui_sink->getQueue().empty()) {
        LoggerDetails details;
        gui_sink->getQueue().pop(details);
        callback(std::move(details));
    }
}

}  // namespace luhsoccer::logger
