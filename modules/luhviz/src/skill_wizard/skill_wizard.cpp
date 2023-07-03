#include "skill_wizard/include/skill_wizard.hpp"
#include <imgui_internal.h>

namespace luhsoccer::luhviz {

void SkillWizard::init() {}

void SkillWizard::render(bool* open) {
    if (!*open) return;

    ImGui::Begin("Skill Wizard", open);
    ImGui::BeginChild("main", {300, 100});

    if (this->create_new_skill) {
        if (ImGui::Button("Back")) {
            this->create_new_skill = false;
            this->edit_skill = false;
        }
    }

    else if (this->edit_skill) {
        if (ImGui::Button("Back")) {
            this->create_new_skill = false;
            this->edit_skill = false;
        }
    }

    else {
        if (ImGui::Button("Edit skill")) {
            this->edit_skill = true;
        }

        if (ImGui::Button("Create new skill")) {
            this->create_new_skill = true;
        }
    }

    ImGui::EndChild();

    ImGui::End();
}

}  // namespace luhsoccer::luhviz