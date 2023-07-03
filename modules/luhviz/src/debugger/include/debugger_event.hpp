#pragma once

#include <utility>
#include "spdlog/details/null_mutex.h"
#include "spdlog/sinks/base_sink.h"
#include <iomanip>
#include <chrono>
#include "imgui.h"

namespace luhsoccer::luhviz {

// This enum contains the several types of Events that might happen
enum class EventType { TRACE, DEBUG, INFO, WARNING, ERROR, CRITICAL, INPUT, NOTE, OFF, OTHER };

// The Event class is meant to contain the data needed to display one event in the console properly
class DebuggerEvent {
   public:
    // ----- members -----
    // ----- methods -----

    // Constructors
    // Note that in this version we move (std::move) the strings, the original data
    // will go into an undefined state

    DebuggerEvent(EventType type, std::string timestamp, std::string module, std::string location, std::string text,
                  ImVec4 color, std::string type_as_string)
        : type(type),
          timestamp(std::move(timestamp)),
          module(std::move(module)),
          location(std::move(location)),
          text(std::move(text)),
          color(color),
          type_as_string(std::move(type_as_string)){};

    // Getters
    [[nodiscard]] EventType getEventType() const { return this->type; }
    [[nodiscard]] std::string getEventTimeStamp() const { return this->timestamp; }
    [[nodiscard]] std::string getEventModule() const { return this->module; }
    [[nodiscard]] std::string getEventLocation() const { return this->location; }
    [[nodiscard]] std::string getEventText() const { return this->text; }
    [[nodiscard]] ImVec4 getEventColor() const { return this->color; }
    [[nodiscard]] std::string getEventTypeAsString() const { return this->type_as_string; }
    [[nodiscard]] int getEventStackCount() const { return this->stack_count; }

    [[nodiscard]] std::string getEventWrappedTextContent() const { return this->wrapped_text_content; }
    [[nodiscard]] std::string getEventOriginalTextContent() const { return this->original_text; }
    [[nodiscard]] float getEventWrappedTextConsoleWidth() const { return this->wrapped_text_console_width; }
    [[nodiscard]] int getEventWrappedTextLineCount() const { return this->wrapped_text_line_count; }

    void setEventWrappedText(std::string wrapped_text, std::string original_text, float console_width, int line_count) {
        this->wrapped_text_content = std::move(wrapped_text);
        this->original_text = std::move(original_text);
        this->wrapped_text_console_width = console_width;
        this->wrapped_text_line_count = line_count;
    }

    // Used to stack the last two events together, can happen multiple times (call only if the events are the same)
    // Increments the event's counter ans updates the timestamp
    void stackEvent(const std::string& timestamp) {
        this->stack_count++;          // Increment counter
        this->timestamp = timestamp;  // Update timestamp
    }

   private:
    // ----- members -----
    EventType type{EventType::OTHER};       // Type of Event (e.g. WARNING or INFO)
    std::string timestamp{"00:00:00:000"};  // Time of Event
    std::string module{""};                 // Source module that triggered the event
    std::string location{""};               // File and line that triggered the event
    std::string text{""};                   // Text contents of Event
    ImVec4 color{ImVec4(1, 1, 1, 1)};       // Color that should be used by the Event
    std::string type_as_string{"OTHER"};    // Type of Event in std::string form

    int stack_count = 1;  // Stores the number of CONSECUTIVE times this event was stacked

    std::string wrapped_text_content = "";  // Stores the wrapped text
    std::string original_text =
        "";  // Stores the original, unwrapped text (used to check if contents have changed since last wrap)
    float wrapped_text_console_width = -1;  // Stores the width of the console for which the text was wrapped
    int wrapped_text_line_count = 1;        // Stores the line count of the wrapped text

    // ----- methods -----
};

}  // namespace luhsoccer::luhviz