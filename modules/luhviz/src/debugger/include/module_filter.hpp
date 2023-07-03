#pragma once

#include "imgui.h"

namespace luhsoccer::luhviz {

// This class keeps track of enabled log types for a single module
class ModuleFilter {
   public:
    // ----- members -----
    bool show_trace = true;     // Keeps track of whether trace logs are enabled or disabled
    bool show_debug = true;     // Keeps track of whether debug logs are enabled or disabled
    bool show_info = true;      // Keeps track of whether info logs are enabled or disabled
    bool show_warning = true;   // Keeps track of whether warning logs are enabled or disabled
    bool show_error = true;     // Keeps track of whether error logs are enabled or disabled
    bool show_critical = true;  // Keeps track of whether critical logs are enabled or disabled
    bool show_off = true;       // Keeps track of whether off logs are enabled or disabled
    bool show_other = true;     // Keeps track of whether other logs are enabled or disabled

    // ----- methods -----
    ModuleFilter(bool show_trace = true, bool show_debug = true, bool show_info = true, bool show_warning = true,
                 bool show_error = true, bool show_critical = true, bool show_off = true, bool show_other = true)
        : show_trace(show_trace),
          show_debug(show_debug),
          show_info(show_info),
          show_warning(show_warning),
          show_error(show_error),
          show_critical(show_critical),
          show_off(show_off),
          show_other(show_other) {}

    void enableAll() {
        this->show_trace = true;
        this->show_debug = true;
        this->show_info = true;
        this->show_warning = true;
        this->show_error = true;
        this->show_critical = true;
        this->show_off = true;
        this->show_other = true;
    }

    void disableAll() {
        this->show_trace = false;
        this->show_debug = false;
        this->show_info = false;
        this->show_warning = false;
        this->show_error = false;
        this->show_critical = false;
        this->show_off = false;
        this->show_other = false;
    }

    void invertSelection() {
        this->show_trace = !this->show_trace;
        this->show_debug = !this->show_debug;
        this->show_info = !this->show_info;
        this->show_warning = !this->show_warning;
        this->show_error = !this->show_error;
        this->show_critical = !this->show_critical;
        this->show_off = !this->show_off;
        this->show_other = !this->show_other;
    }

   private:
    // ----- members -----
    // ----- methods -----
};

}  // namespace luhsoccer::luhviz