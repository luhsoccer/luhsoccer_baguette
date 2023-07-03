#pragma once

#include "imgui.h"

namespace luhsoccer::luhviz {
class SkillWizard {
   public:
    // ----- members -----
    // ----- methods -----
    void init();
    void render(bool* open);

   private:
    // ----- members -----
    bool create_new_skill{false};
    bool edit_skill{false};
    // ----- methods -----
};
}  // namespace luhsoccer::luhviz