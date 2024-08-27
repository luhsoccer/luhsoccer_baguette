#include "include/debugger.hpp"
#include "imgui.h"
#include <cstdlib>
#include <filesystem>
#include "imgui.h"
#include "imgui_stdlib.h"

#include <cstdlib>
#include <filesystem>
#include "imgui.h"
#include <cstdlib>
#include <filesystem>

namespace {
// Takes a time point as a parameter and returns a string with the format "HH:MM:SS:mmm"
std::string formatTime(std::chrono::system_clock::time_point t) {
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch());

    // ms is currently the total of milliseconds, we need to find the "pure" number of milliseconds,
    // so we need to calculate how many seconds are contained in ms and subtract them from ms.
    auto secs = duration_cast<std::chrono::seconds>(ms);
    ms -= duration_cast<std::chrono::milliseconds>(secs);

    // Now we just create a string stream, put in the data and return the stream's contents
    std::stringstream ss;
    time_t temp = std::chrono::system_clock::to_time_t(t);
    ss << std::put_time(std::localtime(&temp), "%H:%M:%S") << ":" << std::setfill('0') << std::setw(3) << ms.count();

    // TODO: Use localtime_s instead of localtime (possibly thread/buffer unsafe or deprecated)

    return ss.str();
}

std::string getCurrentTimeAsTimestamp() {
    auto t = std::chrono::system_clock::now();
    return formatTime(t);
}
}  // namespace

namespace luhsoccer::luhviz {

Debugger::Debugger(Fonts& fonts, DataProxy& proxy) : proxy(proxy), fonts(fonts) {}

void Debugger::baguetteMode() {
    std::string cat = this->baguette_mode_frames[this->baguette_mode_frame];

    switch (this->baguette_mode_frame) {
        case 7:  // If current frame is the maximum (7) reset it to 0
            if (this->baguette_mode_frame_advance) this->baguette_mode_frame = 0;
            break;
        default:  // If current frame is less than the maximum (7) advance it
            if (this->baguette_mode_frame_advance) this->baguette_mode_frame++;
            break;
    }

    unsigned int cur_phase = this->baguette_mode_color_phase;

    // See debugger.hpp for more info on the color phases
    if (cur_phase == 0) {
        if (this->baguette_mode_color_green >= 1.00f) {
            this->baguette_mode_color_green = 1.00f;
            this->baguette_mode_color_phase++;
        } else {
            this->baguette_mode_color_green += this->baguette_mode_color_increment;
        }
    } else if (cur_phase == 1) {
        if (this->baguette_mode_color_red <= 0.00f) {
            this->baguette_mode_color_red = 0.00f;
            this->baguette_mode_color_phase++;
        } else {
            this->baguette_mode_color_red -= this->baguette_mode_color_increment;
        }
    } else if (cur_phase == 2) {
        if (this->baguette_mode_color_blue >= 1.00f) {
            this->baguette_mode_color_blue = 1.00f;
            this->baguette_mode_color_phase++;
        } else {
            this->baguette_mode_color_blue += this->baguette_mode_color_increment;
        }
    } else if (cur_phase == 3) {
        if (this->baguette_mode_color_green <= 0.00f) {
            this->baguette_mode_color_green = 0.00f;
            this->baguette_mode_color_phase++;
        } else {
            this->baguette_mode_color_green -= this->baguette_mode_color_increment;
        }
    } else if (cur_phase == 4) {
        if (this->baguette_mode_color_red >= 1.00f) {
            this->baguette_mode_color_red = 1.00f;
            this->baguette_mode_color_phase++;
        } else {
            this->baguette_mode_color_red += this->baguette_mode_color_increment;
        }
    } else if (cur_phase == 5) {
        if (this->baguette_mode_color_blue <= 0.00f) {
            this->baguette_mode_color_blue = 0.00f;
            this->baguette_mode_color_phase = 0;
        } else {
            this->baguette_mode_color_blue -= this->baguette_mode_color_increment;
        }
    }

    ImVec4 color(this->baguette_mode_color_red, this->baguette_mode_color_green, this->baguette_mode_color_blue, 1);

    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 5);
    ImGui::BeginChild("CatScrollingRegion", ImVec2(0, -0), false, ImGuiWindowFlags_HorizontalScrollbar);
    ImGui::PushFont(fonts.getFont(fonts.FONT_BAGUETTE_MODE));
    ImGui::TextColored(color, "%s", cat.c_str());
    ImGui::PopFont();
    ImGui::EndChild();

    this->baguette_mode_frame_advance = !this->baguette_mode_frame_advance;
}

void Debugger::addEventFromLogger(const logger::LoggerDetails& log) {
    // Convert log to DebuggerEvent

    const ImVec4 grey{0.74f, 0.72f, 0.72f, 1};

    EventType type = EventType::OTHER;
    ImVec4 color = ImVec4(1, 1, 1, 1);  // Remains white unless explicitly changed
    std::string type_as_string;

    if (log.lvl == logger::LogLevel::TRACE) {
        type = EventType::TRACE;
        type_as_string = "TRACE";
        color = ImVec4(0, 1, 0, 1);  // Green (lime)
    } else if (log.lvl == logger::LogLevel::DEBUG) {
        type = EventType::DEBUG;
        type_as_string = "DEBUG";
        // color = ImVec4(0.35f, 0.35f, 0.35f, 1);  // Grey
        color = grey;  // Grey
        // color = ImVec4(0.5f, 0.5f, 0.5f, 1);  // Grey
    } else if (log.lvl == logger::LogLevel::INFO) {
        type = EventType::INFO;
        type_as_string = "INFO";
    } else if (log.lvl == logger::LogLevel::WARNING) {
        type = EventType::WARNING;
        type_as_string = "WARNING";
        color = ImVec4(1, 1, 0, 1);  // Yellow
    } else if (log.lvl == logger::LogLevel::ERROR) {
        type = EventType::ERROR;
        type_as_string = "ERROR";
        color = ImVec4(1, 0, 0, 1);  // Red
    } else if (log.lvl == logger::LogLevel::OFF) {
        type = EventType::OFF;
        type_as_string = "OFF";
    } else {
        type = EventType::OTHER;
        type_as_string = "OTHER";
    }

    std::string formatted_time = formatTime(log.time);

    DebuggerEvent event =
        DebuggerEvent(type, formatted_time, std::string(log.logger_name.data(), log.logger_name.size()), log.location,
                      std::string(log.payload.data(), log.payload.size()), color, type_as_string);

    // If this is not the first event, check if event stacking is possible (if the last events are the same) to
    // improve performance
    if (this->event_log.size() > 0) {
        DebuggerEvent& last_event = this->event_log[this->event_log.size() - 1];

        // Check if last event and new event are the same
        if (last_event.getEventType() == event.getEventType() &&
            last_event.getEventModule() == event.getEventModule() &&
            last_event.getEventLocation() == event.getEventLocation() &&
            last_event.getEventText() == event.getEventText()) {
            // Stack them if they are the same event
            // last_event.stackEvent(std::move(formatted_time));
            this->event_log[this->event_log.size() - 1].stackEvent(event.getEventTimeStamp());
            return;

        } else {
            // If last two events are not the same do not stack them
            addEvent(event);
        }
    } else {
        // If this is the first event just add it normally
        addEvent(event);
    }
}

void Debugger::wrapText(DebuggerEvent& event) {
    if (this->use_monospace_font) {
        ImGui::PushFont(fonts.getFont(fonts.FONT_DEBUGGER_MONO));
    } else {
        ImGui::PushFont(fonts.getFont(fonts.FONT_DEBUGGER_MAIN));
    }

    std::string text = formatEventToString(event);

    // Only do it if the current event has not had its text wrapped before or if the window width has changed!
    // (Or if the event's contents have changed)
    // That is what the if-statement is for
    if (event.getEventWrappedTextContent() == "" ||
        event.getEventWrappedTextConsoleWidth() != ImGui::GetContentRegionAvail().x ||
        event.getEventOriginalTextContent() != text || this->previous_monospace_setting != this->use_monospace_font) {
        std::string result = "";
        int line_count = 1;

        // Split string with spaces as the delimiter
        std::vector<std::string> words;
        std::stringstream ss(text);
        std::string cur_word;
        while (getline(ss, cur_word, ' ')) {
            words.push_back(cur_word);
        }

        int i = 0;

        for (const std::string& word : words) {
            if (i > 0) {
                std::string t = result + " ";
                t += word;
                if (ImGui::CalcTextSize(t.c_str()).x >= ImGui::GetContentRegionAvail().x) {
                    result.append("\n" + word);
                    line_count++;
                } else {
                    result.append(" " + word);
                }
            } else {
                result.append(word);
            }
            i++;
        }

        event.setEventWrappedText(result, text, ImGui::GetContentRegionAvail().x, line_count);
    }

    ImGui::PopFont();
}

void Debugger::printEventAsTextColored(DebuggerEvent& event, int index) {
    if (this->use_monospace_font) {
        ImGui::PushFont(fonts.getFont(fonts.FONT_DEBUGGER_MONO));
    } else {
        ImGui::PushFont(fonts.getFont(fonts.FONT_DEBUGGER_MAIN));
    }

    ImGui::PushID(index);

    std::string text = formatEventToString(event);
    ImGui::TextColored(event.getEventColor(), "%s", text.c_str());

    ImGui::PopID();
    ImGui::PopFont();
}

void Debugger::printEventAsInputText(DebuggerEvent& event, int index) {
    /*if (this->use_monospace_font) {
        ImGui::PushFont(fonts.getFont(fonts.FONT_DEBUGGER_MONO));
    } else {
        ImGui::PushFont(fonts.getFont(fonts.FONT_DEBUGGER_MAIN));
    }

    static ImGuiInputTextFlags flags = ImGuiInputTextFlags_ReadOnly | ImGuiInputTextFlags_CallbackResize;

    ImGui::PushID(index);
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2{0, 0});
    ImGui::PushStyleColor(ImGuiCol_Text, event.getEventColor());
    ImGui::PushStyleColor(ImGuiCol_FrameBg, IM_COL32(37, 37, 38, 255));

    if (this->word_wrap_text) {
        ImGui::PushItemWidth(-1);
        if (this->add_extra_spacing_between_logs) {
            ImVec2 size(-FLT_MIN,
                        (ImGui::GetTextLineHeight() * (float)event.getEventWrappedTextLineCount() + EXTRA_SPACING));
            std::string wrapped_text = event.getEventWrappedTextContent();
            ImGui::InputTextMultiline("##EventDisplay", &wrapped_text, size, flags);
        } else {
            ImVec2 size(-FLT_MIN, (ImGui::GetTextLineHeight() * (float)event.getEventWrappedTextLineCount()));
            std::string wrapped_text = event.getEventWrappedTextContent();
            ImGui::InputTextMultiline("##EventDisplay", &wrapped_text, size, flags);
        }
        ImGui::PopItemWidth();

    } else {
        if (this->add_extra_spacing_between_logs) {
            ImVec2 size(-FLT_MIN, (ImGui::GetTextLineHeight() + EXTRA_SPACING));
            std::string text = formatEventToString(event);
            if (index > 0) ImGui::Dummy(ImVec2(1, EXTRA_SPACING));
            ImGui::PushItemWidth(ImGui::CalcTextSize(text.c_str()).x + 10);
            ImGui::InputText("##EventDisplay", &text);
            ImGui::PopItemWidth();
        } else {
            ImVec2 size(-FLT_MIN, (ImGui::GetTextLineHeight()));
            std::string text = formatEventToString(event);
            ImGui::PushItemWidth(ImGui::CalcTextSize(text.c_str()).x + 10);
            ImGui::InputText("##EventDisplay", &text);
            ImGui::PopItemWidth();
        }
    }

    ImGui::PopID();
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
    ImGui::PopStyleColor();
    ImGui::PopFont();

    // if (this->autoscroll) ImGui::SetScrollHereY(1.0f);
    */
}

std::string Debugger::formatEventToString(const DebuggerEvent& event) const {
    std::string text = "";
    const std::string bracket_open = "[";
    const std::string bracket_close = "] ";

    if (this->show_log_type) {
        text += (bracket_open + event.getEventTypeAsString() + bracket_close);
    }

    if (this->show_timestamps) {
        if (this->show_milliseconds) {
            text += (bracket_open + event.getEventTimeStamp() + bracket_close);
        } else {  // "trim" milliseconds from output
            constexpr int COUNT = 8;
            text += (bracket_open + event.getEventTimeStamp().substr(0, COUNT) + bracket_close);
        }
    }

    if (this->show_module) {
        if (event.getEventModule() != "") {
            text += (bracket_open + event.getEventModule() + bracket_close);
        }
    }

    if (this->show_location) {
        if (event.getEventLocation() != "") {
            text += (bracket_open + event.getEventLocation() + bracket_close);
        }
    }

    if (event.getEventStackCount() > 1) {  // If event has been stacked print the count together with the text
        text += ((event.getEventText() + " (" + std::to_string(event.getEventStackCount()) + "x)"));
    } else {  // else print only the text
        text += event.getEventText();
    };

    return text;
}

void Debugger::addEvent(const DebuggerEvent& event) {
    if (this->MAX_LOG_SIZE == 0 || this->event_log.size() < this->MAX_LOG_SIZE) {
        this->event_log.emplace_back(event);  // If max log size has not been set, just add event normally
    } else {
        // If max log size has been set and reached, delete the oldest event
        this->event_log.erase(this->event_log.begin());
        // if (this->event_heights.size() > 0) this->event_heights.erase(this->event_heights.begin());
        this->event_log.emplace_back(event);  // Then add the new event normally
        // this->event_heights.emplace_back(0);
    }

    // If this module is not included in the module filter map, add it to the module list and the map
    if (this->module_filter_map.find(event.getEventModule()) == this->module_filter_map.end()) {
        this->module_list.emplace_back(event.getEventModule());
        std::sort(this->module_list.begin() + 1, this->module_list.end());
        this->module_filter_map.insert(std::pair<std::string, ModuleFilter>(event.getEventModule(), ModuleFilter()));
    }
}

void Debugger::drawConsole() {
    // const float footer_height_to_reserve = 205;
    const float offset = 5;
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset);
    if (ImGui::BeginChild("ScrollingRegion", ImVec2(0, -FLT_MIN), false, ImGuiWindowFlags_HorizontalScrollbar)) {
        bool new_logs_available = false;  // Tracks whether there is a new log or not

        logger::readLogMessages([&](const logger::LoggerDetails& line) {
            new_logs_available = true;
            addEventFromLogger(line);
        });

        this->events_shown = 0;
        this->filtered_event_log.clear();

        for (DebuggerEvent& e : this->event_log) {
            // Create copies so as to not affect the original data when making it lowercase
            std::string event_text(e.getEventText());
            std::string event_module(e.getEventModule());
            std::string event_location(e.getEventLocation());

            bool content_ok = false;  // Checks whether the event should be printed
                                      // (content-wise, based on filters)
            bool type_ok = false;     // Check whether the event should be printed (type-wise,
                                      // based on the "All modules" type filter)
            bool module_ok = false;   // Check whether the event should be printed (type-wise,
                                      // based on the individual module type filter)

            // Check if event's content passes the search filter
            if (this->filter == "") {
                content_ok = true;
            } else {
                // Case insensitive approach, make everything lowercase first before comparing
                std::transform(this->filter.begin(), this->filter.end(), this->filter.begin(), ::tolower);
                std::transform(event_text.begin(), event_text.end(), event_text.begin(), ::tolower);
                std::transform(event_module.begin(), event_module.end(), event_module.begin(), ::tolower);
                std::transform(event_location.begin(), event_location.end(), event_location.begin(), ::tolower);

                if (event_text.find(this->filter) != std::string::npos) {
                    content_ok = true;
                }

                if (event_module.find(this->filter) != std::string::npos) {
                    content_ok = true;
                }

                if (event_location.find(this->filter) != std::string::npos) {
                    content_ok = true;
                }
            }

            // Check if event's type passes the type filter
            EventType type = e.getEventType();

            // Check if the event has a module assigned to it (it may be a "Welcome" event, etc. and have no module!!)
            bool module_is_null = (this->module_filter_map.find(e.getEventModule()) == this->module_filter_map.end());
            if (module_is_null) {
                module_ok = true;

                if (type == EventType::TRACE) {
                    if (this->show_trace) type_ok = true;
                } else if (type == EventType::DEBUG) {
                    if (this->show_debug) type_ok = true;
                } else if (type == EventType::INFO) {
                    if (this->show_info) type_ok = true;
                } else if (type == EventType::WARNING) {
                    if (this->show_warning) type_ok = true;
                } else if (type == EventType::ERROR) {
                    if (this->show_error) type_ok = true;
                } else if (type == EventType::CRITICAL) {
                    if (this->show_critical) type_ok = true;
                } else if (type == EventType::OFF) {
                    if (this->show_off) type_ok = true;
                } else if (type == EventType::OTHER) {
                    if (this->show_other) type_ok = true;
                }
            } else {
                ModuleFilter* module_filter = &this->module_filter_map.find(e.getEventModule())->second;
                type_ok = true;
                if (type == EventType::TRACE) {
                    if (this->show_trace) type_ok = true;
                    if (module_filter->show_trace) module_ok = true;
                } else if (type == EventType::DEBUG) {
                    if (this->show_debug) type_ok = true;
                    if (module_filter->show_debug) module_ok = true;
                } else if (type == EventType::INFO) {
                    if (this->show_info) type_ok = true;
                    if (module_filter->show_info) module_ok = true;
                } else if (type == EventType::WARNING) {
                    if (this->show_warning) type_ok = true;
                    if (module_filter->show_warning) module_ok = true;
                } else if (type == EventType::ERROR) {
                    if (this->show_error) type_ok = true;
                    if (module_filter->show_error) module_ok = true;
                } else if (type == EventType::CRITICAL) {
                    if (this->show_critical) type_ok = true;
                    if (module_filter->show_critical) module_ok = true;
                } else if (type == EventType::OFF) {
                    if (this->show_off) type_ok = true;
                    if (module_filter->show_off) module_ok = true;
                } else if (type == EventType::OTHER) {
                    if (this->show_other) type_ok = true;
                    if (module_filter->show_other) module_ok = true;
                }
            }

            // Print event only if both content and type pass the respective filters
            if (content_ok && type_ok && module_ok) {
                if (this->word_wrap_text) {
                    // We attempt to wrap the text
                    // It will only happen if the width of the console has been changed
                    // or if the text has not been wrapped before,
                    // otherwise it will be automatically cancelled, so it (hopefully) cannot tank the performance
                    this->wrapText(e);
                }
                this->filtered_event_log.push_back(e);
                this->events_shown++;
            }

            // If for any reason the new amount of shown events is different than the old one,
            // it means that the user either changed the filtering criteria or that there are new logs,
            // so we set new_logs_available to true so that the content will autoscroll to the bottom
            // if autoscroll is enabled as well
        }
        if (this->events_shown != this->previous_events_shown) {
            // std::cout << this->events_shown << "," << this->previous_events_shown << std::endl;
            new_logs_available = true;
        }

        this->previous_events_shown = this->events_shown;

        // this->event_heights.clear();

        /*for (DebuggerEvent& e : this->filtered_event_log) {
            if (this->word_wrap_text) {
                this->event_heights.push_back(ImGui::CalcTextSize(e.getEventWrappedTextContent().c_str()).y);
            } else {
                this->event_heights.push_back(ImGui::GetTextLineHeight());
            }
        }*/

        // Autoscroll to bottom only if a new log is available and the user has enabled
        // autoscroll
        // this->printEvents(new_logs_available);
        this->printEventsRewritten(new_logs_available);

        // Finally, update the previous monospace setting for the next print cycle (see debugger.hpp for more details)
        this->previous_monospace_setting = this->use_monospace_font;
    }
    ImGui::EndChild();
}

void Debugger::updateAllModulesLogTypeConfig() {
    this->proxy.setConfigBool("show_trace", this->show_trace);
    this->proxy.setConfigBool("show_debug", this->show_debug);
    this->proxy.setConfigBool("show_info", this->show_info);
    this->proxy.setConfigBool("show_warning", this->show_warning);
    this->proxy.setConfigBool("show_error", this->show_error);
    this->proxy.setConfigBool("show_critical", this->show_critical);
    this->proxy.setConfigBool("show_off", this->show_off);
    this->proxy.setConfigBool("show_other", this->show_other);
}

void Debugger::loadConfigs() {
    // Load options
    this->autoscroll = this->proxy.getConfigBool(config_name, "autoscroll");
    this->show_log_type = this->proxy.getConfigBool(config_name, "show_log_type");
    this->show_timestamps = this->proxy.getConfigBool(config_name, "show_timestamps");
    this->show_milliseconds = this->proxy.getConfigBool(config_name, "show_milliseconds");
    this->show_module = this->proxy.getConfigBool(config_name, "show_module");
    this->show_location = this->proxy.getConfigBool(config_name, "show_location");
    this->word_wrap_text = this->proxy.getConfigBool(config_name, "word_wrap_text");
    this->use_monospace_font = this->proxy.getConfigBool(config_name, "use_monospace_font");
    this->previous_monospace_setting = this->use_monospace_font;
    this->add_extra_spacing_between_logs = this->proxy.getConfigBool(config_name, "add_extra_spacing_between_logs");
    this->baguette_mode = this->proxy.getConfigBool(config_name, "baguette_mode");

    // Load module filters for ALL modules
    this->show_trace = this->proxy.getConfigBool(config_name, "show_trace");
    this->show_debug = this->proxy.getConfigBool(config_name, "show_debug");
    this->show_info = this->proxy.getConfigBool(config_name, "show_info");
    this->show_warning = this->proxy.getConfigBool(config_name, "show_warning");
    this->show_error = this->proxy.getConfigBool(config_name, "show_error");
    this->show_critical = this->proxy.getConfigBool(config_name, "show_critical");
    this->show_off = this->proxy.getConfigBool(config_name, "show_off");
    this->show_other = this->proxy.getConfigBool(config_name, "show_other");

    // Load module filters for individual modules
    std::string module_filters = this->proxy.getConfigString(config_name, "module_filters");

    std::stringstream ss(module_filters);
    std::vector<std::vector<std::string>> modules;
    std::string module;

    while (std::getline(ss, module, '|')) {
        std::stringstream m_ss(module);
        std::string temp;
        std::vector<std::string> m;
        while (std::getline(m_ss, temp, '-')) {
            m.emplace_back(temp);
        }
        modules.emplace_back(m);
    }

    for (const auto& m : modules) {
        assert(m.size() == 9);

        this->module_list.emplace_back(m[0]);

        bool show_trace = (std::stoi(m[1]) == 1) ? true : false;
        bool show_debug = (std::stoi(m[2]) == 1) ? true : false;
        bool show_info = (std::stoi(m[3]) == 1) ? true : false;
        bool show_warning = (std::stoi(m[4]) == 1) ? true : false;
        bool show_error = (std::stoi(m[5]) == 1) ? true : false;
        bool show_critical = (std::stoi(m[6]) == 1) ? true : false;
        bool show_off = (std::stoi(m[7]) == 1) ? true : false;
        bool show_other = (std::stoi(m[8]) == 1) ? true : false;

        ModuleFilter mf(show_trace, show_debug, show_info, show_warning, show_error, show_critical, show_off,
                        show_other);

        this->module_filter_map.insert(std::pair<std::string, ModuleFilter>(m[0], mf));
    }

    std::sort(this->module_list.begin() + 1, this->module_list.end());
}

void Debugger::printEventsRewritten(bool new_logs_available) {
    for (size_t index = 0; index < this->filtered_event_log.size(); index++) {
        this->printEventAsTextColored(this->filtered_event_log.at(index), (int)index);

        if (ImGui::IsItemHovered() && ImGui::IsMouseReleased(ImGuiMouseButton_Right)) {
            // Open the context menu to copy the log to the clipboard
            ImGui::OpenPopup(("LogContextMenu" + std::to_string(index)).c_str());
        }

        // Context menu for each log
        if (ImGui::BeginPopupContextItem(("LogContextMenu" + std::to_string(index)).c_str())) {
            if (ImGui::MenuItem("Copy log to clipboard")) {
                // Copy a specific log to the clipboard
                ImGui::SetClipboardText(this->filtered_event_log.at(index).getEventText().c_str());
            }

            ImGui::EndPopup();
        }
    }

    if (this->autoscroll && new_logs_available) ImGui::SetScrollY(ImGui::GetScrollMaxY());
}

void Debugger::printEvents(bool new_logs_available) {
    // Currently disabled as it is buggy
    // If re-enabling it DO NOT FORGET to also remove ALL commented event_heights code in debugger.cpp and debugger.hpp

    /*ImGuiListClipper clipper;

    size_t item_current = 0;
    float item_current_y = 0.0f;
    float total_height = 0;

    float spacing = 0;
    if (this->add_extra_spacing_between_logs) spacing = EXTRA_SPACING;

    if (this->word_wrap_text) {
        for (float height : this->event_heights) {
            if (!this->add_extra_spacing_between_logs) {
                total_height += height;
            } else {
                total_height += height + EXTRA_SPACING;
            }
        }
    } else {
        total_height = (float)(this->event_heights.size()) * (ImGui::GetTextLineHeight() + spacing);
    }

    clipper.Begin((int)total_height, 1.0f);

    assert(this->event_heights.size() == this->filtered_event_log.size());

    item_current = this->filtered_event_log.size() - 1;
    item_current_y = total_height;

    while (clipper.Step()) {
        auto pos_min_y = (float)clipper.DisplayStart;
        auto pos_max_y = (float)clipper.DisplayEnd;

        size_t item_max = this->filtered_event_log.size() - 1;

        while (item_current_y > pos_max_y) {
            item_max = item_current;

            if (this->word_wrap_text) {
                item_current_y -= (this->event_heights[item_current] + spacing);
            } else {
                item_current_y -= (ImGui::GetTextLineHeight() + spacing);
            }

            item_current--;
        }

        size_t item_min = item_max;

        while (item_current_y > pos_min_y) {
            item_min = item_current;

            if (this->word_wrap_text) {
                item_current_y -= (this->event_heights[item_current] + spacing);
            } else {
                item_current_y -= (ImGui::GetTextLineHeight() + spacing);
            }

            item_current--;
        }

        float difference = abs(item_current_y - pos_min_y);

        ImGui::SetCursorPosY(pos_min_y - difference);

        for (size_t index = item_min; index <= item_max; index++) {
            if (index >= this->filtered_event_log.size()) break;
            this->printEventAsInputText(this->filtered_event_log.at(index), (int)index);
        }
    }

    if (this->autoscroll && new_logs_available) ImGui::SetScrollY(total_height);
    clipper.End();
    */
}

void Debugger::drawWindow(ImGuiWindowFlags& flags, bool& open) {
    if (!open) {
        return;
    }

    // Initialize draggable-window
    const ImVec2 window_size{520, 600};
    const ImVec2 window_min_size{250, 250};
    const float offset = 5;

    ImGui::SetNextWindowSize(window_size, ImGuiCond_FirstUseEver);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowMinSize, window_min_size);

    ImGui::PushStyleColor(ImGuiCol_Text, this->proxy.accent_text_color);
    ImGui::Begin(this->title.c_str(), &open, flags);
    ImGui::PopStyleColor();

    // Note: As a specific feature guaranteed by the library, after calling Begin() the last
    // Item represents the title bar.

    // Makeshift "padding"
    ImGui::SameLine(offset);
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset);

    // Setting up the Options menu
    if (ImGui::BeginPopup("Options")) {
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset);

        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset);
        if (ImGui::Checkbox("Auto-Scroll  ", &this->autoscroll)) {
            this->proxy.setConfigBool("autoscroll", this->autoscroll);
        }

        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset);
        if (ImGui::Checkbox("Display log types  ", &this->show_log_type)) {
            this->proxy.setConfigBool("show_log_type", this->show_log_type);
        }

        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset);
        if (ImGui::Checkbox("Display timestamps  ", &this->show_timestamps)) {
            this->proxy.setConfigBool("show_timestamps", this->show_timestamps);
        }

        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset);
        if (ImGui::Checkbox("Display milliseconds in timestamps  ", &this->show_milliseconds)) {
            this->proxy.setConfigBool("show_milliseconds", this->show_milliseconds);
        }

        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset);
        if (ImGui::Checkbox("Display source module  ", &this->show_module)) {
            this->proxy.setConfigBool("show_module", this->show_module);
        }

        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset);
        if (ImGui::Checkbox("Display source file and line  ", &this->show_location)) {
            this->proxy.setConfigBool("show_location", this->show_location);
        }

        /*ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset);
        if (ImGui::Checkbox("Word-wrap text (may reduce performance)  ", &this->word_wrap_text)) {
            this->proxy.setConfigBool("word_wrap_text", this->word_wrap_text);
        }*/

        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset);
        if (ImGui::Checkbox("Use monospace font  ", &this->use_monospace_font)) {
            this->proxy.setConfigBool("use_monospace_font", this->use_monospace_font);
        }

        /*ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset);
        if (ImGui::Checkbox("Add extra spacing between logs  ", &this->add_extra_spacing_between_logs)) {
            this->proxy.setConfigBool("add_extra_spacing_between_logs", this->add_extra_spacing_between_logs);
        }*/

        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset);
        if (ImGui::Checkbox("Baguette mode  ", &this->baguette_mode)) {
            this->proxy.setConfigBool("baguette_mode", this->baguette_mode);
        }

        if (!this->event_log.empty()) {
            ImGui::Separator();
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 2);
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset);

            if (ImGui::Button("Clear logs")) {
                this->event_log.clear();
                this->filtered_event_log.clear();
            }
            // Set tooltip for the button - it is guaranteed to be the last item for
            // IsItemHovered() For some reason clang rejects the SetTooltip method because it is
            // "variadic" Do we just ignore the warning?
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Click to clear all logged events so far");
            }

            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 2);
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset);
            ImGui::SameLine();

            if (ImGui::Button("Copy all logs to clipboard")) {
                std::string log_output = "";
                for (const auto& e : this->event_log) {
                    log_output += formatEventToString(e) + "\n";
                    ImGui::SetClipboardText(log_output.c_str());
                }
                // Set tooltip for the button - it is guaranteed to be the last item for
                // IsItemHovered() For some reason clang rejects the SetTooltip method because it
                // is "variadic" Do we just ignore the warning?
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Click to copy all stored events (ignoring filters) to the clipboard");
                }
            }
        }

        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset);
        ImGui::EndPopup();
    }

    if (ImGui::Button("Options")) ImGui::OpenPopup("Options");

    if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("Click to open the log display options");
    }

    if (!this->baguette_mode) {
        // Setting up the FilterTypes menu
        const float offset_big = 10;
        if (ImGui::BeginPopup("FilterTypes")) {
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset_big);
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset_big);

            auto items = this->module_list;

            static std::string selected_module = "All modules";
            static ModuleFilter* selected_module_filter;
            static int combo_selected_module_index = 0;

            if (ImGui::Combo(
                    "##Combobox", &combo_selected_module_index,
                    [](void* vec, int idx, const char** out_text) {
                        auto& vector = *static_cast<std::vector<std::string>*>(vec);
                        *out_text = vector.at(idx).c_str();
                        return true;
                    },
                    static_cast<void*>(&items), static_cast<int>(items.size()))) {
                selected_module = this->module_list.at(combo_selected_module_index);

                selected_module_filter = &this->module_filter_map.find(selected_module)->second;
            }

            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset);
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset_big);

            if (selected_module == "All modules") {
                ImGui::TextColored(ImVec4(1, 1, 0, 1),
                                   "Note: This settings change the values in\n"
                                   "all modules!");

                ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset);
                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset_big);

                // Enables all checkboxes
                if (ImGui::Button("Enable all")) {
                    this->show_trace = true;
                    this->show_debug = true;
                    this->show_info = true;
                    this->show_warning = true;
                    this->show_error = true;
                    this->show_critical = true;
                    this->show_off = true;
                    this->show_other = true;

                    updateAllModulesLogTypeConfig();
                    for (auto& [module, filter] : this->module_filter_map) {
                        filter.enableAll();
                        updateModuleFilter(module, &filter);
                    }
                }

                ImGui::SameLine();

                // Disables all checkboxes
                if (ImGui::Button("Disable all")) {
                    this->show_trace = false;
                    this->show_debug = false;
                    this->show_info = false;
                    this->show_warning = false;
                    this->show_error = false;
                    this->show_critical = false;
                    this->show_off = false;
                    this->show_other = false;

                    updateAllModulesLogTypeConfig();
                    for (auto& [module, filter] : this->module_filter_map) {
                        filter.disableAll();
                        updateModuleFilter(module, &filter);
                    }
                }

                ImGui::SameLine();

                // Inverts checkboxes (enabled ones will be disabled and vice versa)
                if (ImGui::Button("Invert selection")) {
                    this->show_trace = !this->show_trace;
                    this->show_debug = !this->show_debug;
                    this->show_info = !this->show_info;
                    this->show_warning = !this->show_warning;
                    this->show_error = !this->show_error;
                    this->show_critical = !this->show_critical;
                    this->show_off = !this->show_off;
                    this->show_other = !this->show_other;

                    updateAllModulesLogTypeConfig();
                    for (auto& [module, filter] : this->module_filter_map) {
                        filter.invertSelection();
                        updateModuleFilter(module, &filter);
                    }
                }

                // This invisible text is simply there for padding
                ImGui::SameLine();
                ImGui::Text("  ");

                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset_big);
                ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset);
                ImGui::BeginTable("types_table", 2);

                ImGui::TableNextRow();  // Needs to be called to create a row, so even for the first
                                        // row

                ImGui::TableSetColumnIndex(0);
                if (ImGui::Checkbox("TRACE  ", &this->show_trace)) {
                    this->proxy.setConfigBool("show_trace", this->show_trace);
                    for (auto& [module, filter] : this->module_filter_map) {
                        filter.show_trace = this->show_trace;
                        updateModuleFilter(module, &filter);
                    }
                }
                ImGui::TableSetColumnIndex(1);
                if (ImGui::Checkbox("ERROR  ", &this->show_error)) {
                    this->proxy.setConfigBool("show_error", this->show_error);
                    for (auto& [module, filter] : this->module_filter_map) {
                        filter.show_error = this->show_error;
                        updateModuleFilter(module, &filter);
                    }
                }

                ImGui::TableNextRow();

                ImGui::TableSetColumnIndex(0);
                if (ImGui::Checkbox("DEBUG  ", &this->show_debug)) {
                    this->proxy.setConfigBool("show_debug", this->show_debug);
                    for (auto& [module, filter] : this->module_filter_map) {
                        filter.show_debug = this->show_debug;
                        updateModuleFilter(module, &filter);
                    }
                }
                ImGui::TableSetColumnIndex(1);
                if (ImGui::Checkbox("CRITICAL  ", &this->show_critical)) {
                    this->proxy.setConfigBool("show_critical", this->show_critical);
                    for (auto& [module, filter] : this->module_filter_map) {
                        filter.show_critical = this->show_critical;
                        updateModuleFilter(module, &filter);
                    }
                }

                ImGui::TableNextRow();

                ImGui::TableSetColumnIndex(0);
                if (ImGui::Checkbox("INFO  ", &this->show_info)) {
                    this->proxy.setConfigBool("show_info", this->show_info);
                    for (auto& [module, filter] : this->module_filter_map) {
                        filter.show_info = this->show_info;
                        updateModuleFilter(module, &filter);
                    }
                }
                ImGui::TableSetColumnIndex(1);
                if (ImGui::Checkbox("OFF  ", &this->show_off)) {
                    this->proxy.setConfigBool("show_off", this->show_off);
                    for (auto& [module, filter] : this->module_filter_map) {
                        filter.show_off = this->show_off;
                        updateModuleFilter(module, &filter);
                    }
                }

                ImGui::TableNextRow();

                ImGui::TableSetColumnIndex(0);
                if (ImGui::Checkbox("WARNING  ", &this->show_warning)) {
                    this->proxy.setConfigBool("show_warning", this->show_warning);
                    for (auto& [module, filter] : this->module_filter_map) {
                        filter.show_warning = this->show_warning;
                        updateModuleFilter(module, &filter);
                    }
                }
                ImGui::TableSetColumnIndex(1);
                if (ImGui::Checkbox("OTHER  ", &this->show_other)) {
                    this->proxy.setConfigBool("show_other", this->show_other);
                    for (auto& [module, filter] : this->module_filter_map) {
                        filter.show_other = this->show_other;
                        updateModuleFilter(module, &filter);
                    }
                }

                ImGui::EndTable();
            } else {
                // If a specific module has been selected

                // Enables all checkboxes
                if (ImGui::Button("Enable all")) {
                    selected_module_filter->enableAll();
                    updateModuleFilter(selected_module, selected_module_filter);
                }

                ImGui::SameLine();

                // Disables all checkboxes
                if (ImGui::Button("Disable all")) {
                    selected_module_filter->disableAll();
                    updateModuleFilter(selected_module, selected_module_filter);
                }

                ImGui::SameLine();

                // Inverts checkboxes (enabled ones will be disabled and vice versa)
                if (ImGui::Button("Invert selection")) {
                    selected_module_filter->invertSelection();
                    updateModuleFilter(selected_module, selected_module_filter);
                }

                // This invisible text is simply there for padding
                ImGui::SameLine();
                ImGui::Text("  ");

                ImGui::SetCursorPosX(ImGui::GetCursorPosX() + offset_big);
                ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset);
                ImGui::BeginTable("types_table", 2);

                ImGui::TableNextRow();  // Needs to be called to create a row, so even for the first
                                        // row

                ImGui::TableSetColumnIndex(0);
                if (ImGui::Checkbox("TRACE  ", &selected_module_filter->show_trace)) {
                    updateModuleFilter(selected_module, selected_module_filter);
                }
                ImGui::TableSetColumnIndex(1);
                if (ImGui::Checkbox("ERROR  ", &selected_module_filter->show_error)) {
                    updateModuleFilter(selected_module, selected_module_filter);
                }

                ImGui::TableNextRow();

                ImGui::TableSetColumnIndex(0);
                if (ImGui::Checkbox("DEBUG  ", &selected_module_filter->show_debug)) {
                    updateModuleFilter(selected_module, selected_module_filter);
                }
                ImGui::TableSetColumnIndex(1);
                if (ImGui::Checkbox("CRITICAL  ", &selected_module_filter->show_critical)) {
                    updateModuleFilter(selected_module, selected_module_filter);
                }

                ImGui::TableNextRow();

                ImGui::TableSetColumnIndex(0);
                if (ImGui::Checkbox("INFO  ", &selected_module_filter->show_info)) {
                    updateModuleFilter(selected_module, selected_module_filter);
                }
                ImGui::TableSetColumnIndex(1);
                if (ImGui::Checkbox("OFF  ", &selected_module_filter->show_off)) {
                    updateModuleFilter(selected_module, selected_module_filter);
                }

                ImGui::TableNextRow();

                ImGui::TableSetColumnIndex(0);
                if (ImGui::Checkbox("WARNING  ", &selected_module_filter->show_warning)) {
                    updateModuleFilter(selected_module, selected_module_filter);
                }
                ImGui::TableSetColumnIndex(1);
                if (ImGui::Checkbox("OTHER  ", &selected_module_filter->show_other)) {
                    updateModuleFilter(selected_module, selected_module_filter);
                }

                ImGui::EndTable();
            }

            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset);

            ImGui::EndPopup();
        }

        ImGui::SameLine();
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset);
        if (ImGui::Button("Filter logs by type")) ImGui::OpenPopup("FilterTypes");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Include or exclude logs from the console based on their type (e.g. DEBUG)");
        }

        ImGui::SameLine();
        ImGui::PushStyleVar(ImGuiStyleVar_GrabRounding, 0);
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 0);
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset);
        ImGui::PushFont(fonts.getFont(fonts.FONT_DEBUGGER_MAIN));
        const float width_scale_factor = 0.55f;
        ImGui::PushItemWidth(ImGui::GetWindowWidth() * width_scale_factor);
        std::string hint_text = "Type to search in ";
        if (filtered_event_log.size() == 1) {
            hint_text.append("1 event");
        } else {
            hint_text.append(std::to_string(this->filtered_event_log.size())).append(" events");
        }
        ImGui::InputTextWithHint("##SearchInput", hint_text.c_str(), &this->filter);
        ImGui::PopStyleVar();
        ImGui::PopStyleVar();
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Search for logs (case insensitive, also checks the module/file names)");
        }
        ImGui::PopFont();
    }

    if (this->filter != "") {
        ImGui::SameLine(0, 0);
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset);
        ImGui::PushFont(fonts.getFont(fonts.FONT_DEBUGGER_MAIN));
        ImGui::PushStyleVar(ImGuiStyleVar_GrabRounding, 0);
        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 0);
        if (ImGui::Button("x")) {
            this->filter = "";
        }
        ImGui::PopFont();
        ImGui::PopStyleVar();
        ImGui::PopStyleVar();

        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Clear all text in the search field");
        }

        if (!this->baguette_mode) {
            ImGui::SameLine();
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset);
            ImGui::PushFont(fonts.getFont(fonts.FONT_STANDARD));
            std::string events_shown_text("Showing " + std::to_string(this->events_shown) + " out of " +
                                          std::to_string(this->event_log.size()) + " event(s)");
            ImGui::Text("%s", events_shown_text.c_str());
            ImGui::PopFont();
        }

    } else {
        if (!this->baguette_mode) {
            ImGui::SameLine();
            ImGui::SetCursorPosY(ImGui::GetCursorPosY() + offset);
            ImGui::PushFont(fonts.getFont(fonts.FONT_STANDARD));
            if (this->events_shown < static_cast<int>(this->event_log.size())) {
                std::string events_shown_text("Showing " + std::to_string(this->events_shown) + " out of " +
                                              std::to_string(this->event_log.size()) + " event(s)");
                ImGui::Text("%s", events_shown_text.c_str());
            } else {
                std::string events_shown_text(std::to_string(this->event_log.size()) + " event(s) total");
                ImGui::Text("%s", events_shown_text.c_str());
            }
            ImGui::PopFont();
        }
    }

    const float y_offset = 1.5f;
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + y_offset);
    ImGui::Separator();

    if (!this->baguette_mode) {  // If baguette mode is off
        this->drawConsole();
    } else {  // If baguette mode is on
        this->baguetteMode();
    }

    ImGui::End();
    ImGui::PopStyleVar();
}

std::string loadBaguetteModeFrame(int frame, cmrc::embedded_filesystem fs) {
    std::string file_path;
    switch (frame) {
        case 0:
            file_path = "res/cat/cat0.txt";
            break;
        case 1:
            file_path = "res/cat/cat1.txt";
            break;
        case 2:
            file_path = "res/cat/cat2.txt";
            break;
        case 3:
            file_path = "res/cat/cat3.txt";
            break;
        case 4:
            file_path = "res/cat/cat4.txt";
            break;
        case 5:
            file_path = "res/cat/cat5.txt";
            break;
        case 6:
            file_path = "res/cat/cat6.txt";
            break;
        case 7:
            file_path = "res/cat/cat7.txt";
            break;
        default:
            file_path = "res/cat/cat0.txt";  // This should not be reached
    }

    auto file = fs.open(file_path.c_str());
    std::string cat{file.begin(), file.end()};

    return cat;
}

void Debugger::updateModuleFilter(const std::string& module_name, ModuleFilter* module_filter) {
    std::string config = this->proxy.getConfigString(config_name, "module_filters");

    assert(module_filter != nullptr);

    bool show_trace = module_filter->show_trace;
    bool show_debug = module_filter->show_debug;
    bool show_info = module_filter->show_info;
    bool show_warning = module_filter->show_warning;
    bool show_error = module_filter->show_error;
    bool show_critical = module_filter->show_critical;
    bool show_off = module_filter->show_off;
    bool show_other = module_filter->show_other;

    // Find the position of the module's name in the string (if it exists)
    size_t start = config.find(module_name);

    // Check if the module's name was found
    if (start != std::string::npos) {
        // If this has not been stored before, update the config

        std::stringstream ss;

        ss << module_name << "-" << std::to_string(show_trace) << "-" << std::to_string(show_debug) << "-"
           << std::to_string(show_info) << "-" << std::to_string(show_warning) << "-" << std::to_string(show_error)
           << "-" << std::to_string(show_critical) << "-" << std::to_string(show_off) << "-"
           << std::to_string(show_other) << "|";

        std::string str = ss.str();

        // Replace the relevant part of the config string
        config.replace(start, str.length(), str);
        this->proxy.setConfigString("module_filters", str);

    } else {
        // If this has not been stored before, create a new string and add it to the config

        std::stringstream ss;

        ss << module_name << "-" << std::to_string(show_trace) << "-" << std::to_string(show_debug) << "-"
           << std::to_string(show_info) << "-" << std::to_string(show_warning) << "-" << std::to_string(show_error)
           << "-" << std::to_string(show_critical) << "-" << std::to_string(show_off) << "-"
           << std::to_string(show_other) << "|";

        std::string str = ss.str();

        config.append(str);
        this->proxy.setConfigString("module_filters", str);
    }
}

void Debugger::init() {
    // Load configs
    this->loadConfigs();

    // Load baguette mode frames
    auto fs = cmrc::luhviz::get_filesystem();

    for (int i = 0; i < (int)this->baguette_mode_frames.size(); i++) {
        this->baguette_mode_frames[i] = loadBaguetteModeFrame(i, fs);
    }

    // Add welcome event to event log
    const ImVec4 color{0.015f, 0.85f, 1, 1};
    if (this->MAX_LOG_SIZE == 0) {
        this->event_log.emplace_back(EventType::OTHER, getCurrentTimeAsTimestamp(), "", "",
                                     "Welcome to Luhviz! Consider setting an event log limit to improve performance.",
                                     color /*Neon light blue*/, "OTHER");
    } else {
        this->event_log.emplace_back(EventType::OTHER, getCurrentTimeAsTimestamp(), "", "", "Welcome to Luhviz!",
                                     color /*Neon light blue*/, "OTHER");
    }
}

void Debugger::render(bool& open) {
    ImGuiWindowFlags window_flags =
        ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;

    this->drawWindow(window_flags, open);
}

}  // namespace luhsoccer::luhviz