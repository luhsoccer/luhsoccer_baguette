#include "robot_controller/include/robot_controller.hpp"
#include <imgui_internal.h>

namespace luhsoccer::luhviz {

void RobotController::init() {
    this->gamepad_icon.create(gamepad_icon_path, false, true);

    // creates all possible robot identifiers
    const size_t num_robots = 16;
    for (size_t i = 0; i < num_robots; ++i) {
        this->robot_ids.emplace_back(RobotIdentifier{i, Team::ALLY});
    }
    ids_as_string = stringFromIds(this->robot_ids);
}

void RobotController::render(bool* open, const std::vector<size_t>& active_controllers) {
    if (!*open) {
        // disable controlling if this is closed
        for (const auto& [controller, data] : this->proxy.getControllerData()) {
            if (!data.robot_id.has_value()) continue;
            this->proxy.setLocalPlannerDisabledForRobot(data.robot_id.value(), false);
        }
        this->proxy.clearControllerRobotidentifyer();
        return;
    }


    ImGui::PushStyleColor(ImGuiCol_Text, this->proxy.accent_text_color);
    ImGui::Begin("Robot Controller", open, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoDocking);
    ImGui::PopStyleColor();

    // handle new / old controllers
    this->manageControllerRegistration(active_controllers);

    ImGui::SetCursorPosX(10);
    ImGui::Checkbox("Global Movement", &global_movement);

    int i = 0;
    for (const auto& c : active_controllers) {
        displayControllRobotUI(active_controllers, i);
        ++i;
    }

    // Display controller image here to understand the controls and display the current Kick Voltage / funk connection
    // etc.
    constexpr ImVec2 ICON_SIZE{270, 223};
    const float icon_x = 420;
    const float icon_y = 230;
    ImGui::SetCursorPos({icon_x - ICON_SIZE.x / 2, icon_y - ICON_SIZE.y / 3 * 2});
    ImGui::Image(this->gamepad_icon.getImguiId(), ICON_SIZE);

    const float text_x = 280;
    const float text_y = 5;
    ImGui::SetCursorPos({ImGui::GetCursorPosX() + text_x, ImGui::GetCursorPosY() + text_y});
    ImGui::Text(
        "Keyboard control: \n\tWASD -> Movement      \t\t\tQE -> Rotation \n\tSPACE -> dribbler on/off \t\tENTER -> "
        "Kick");

    ImGui::End();
}

void RobotController::displayControllRobotUI(const std::vector<size_t>& active_controllers, size_t i) {
    auto& conti = this->proxy.getControllerData();

    size_t id = active_controllers[i];

    // safety check
    if (conti.find(id) == conti.end()) return;

    auto& conti_data = conti.at(id);

    // Robot id chooser
    const ImVec2 margin{10, 5 + i * 110.0f};
    const float combo_width = 100;
    std::string name = "Keyboard";
    if (active_controllers[i] != 16) {
        name = "Gamepad" + std::to_string(id);
    }
    ImGui::Text(" (%s) robot id: ", name.c_str());
    ImGui::SetCursorPosX(margin.x);
    ImGui::PushItemWidth(combo_width);
    const std::string label{"##robot id: " + std::to_string(i)};

    int selection = 0;
    int current_selection = 0;
    if (conti_data.robot_id.has_value()) {
        current_selection = static_cast<int>(conti_data.robot_id.value().id) + 1;
        selection = current_selection;
    }
    ImGui::Combo(label.c_str(), &selection, this->ids_as_string.c_str(), static_cast<int>(this->robot_ids.size()));
    ImGui::PopItemWidth();

    // check if selection changed
    if (selection != current_selection) {
        // valid robot id selected
        if (selection > 0 && selection < static_cast<int>(this->robot_ids.size() + 1)) {
            // adjust gui offset
            selection--;

            // update robot reference
            conti_data.robot_id = this->robot_ids[selection];

            // disable for localplanner
            this->proxy.setLocalPlannerDisabledForRobot(this->robot_ids[selection], true);

        } else {
            if (conti_data.robot_id.has_value()) {
                this->proxy.setLocalPlannerDisabledForRobot(conti_data.robot_id.value(), false);
            }
            conti_data.robot_id.reset();
        }
    }

    // display kick voltage
    const float offset_x = 150;
    const float item_width = 100;
    const float drag_speed = 0.05;
    const float vel_max = 3.0;
    const float rot_vel_max = 8.0;
    ImGui::SameLine();
    ImGui::SetCursorPosX(offset_x);
    ImGui::Text("Kick-Voltage: %d", this->proxy.getKickVoltage());

    // render velocity settings
    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + margin.x);
    ImGui::Text("Movement velocity:");
    ImGui::SameLine();
    ImGui::SetCursorPosX(offset_x);
    ImGui::SetNextItemWidth(item_width);
    ImGui::DragFloat("##Velocity", &this->proxy.getMovementVelocity(), drag_speed, 0.0, vel_max, "%.2f");

    ImGui::SetCursorPosX(ImGui::GetCursorPosX() + margin.x);
    ImGui::Text("Rotation velocity:");
    ImGui::SameLine();
    ImGui::SetCursorPosX(offset_x);
    ImGui::SetNextItemWidth(item_width);
    ImGui::DragFloat("##RVelocity", &this->proxy.getRotationVelocity(), drag_speed, 0.0, rot_vel_max, "%.2f");
}

void RobotController::manageControllerRegistration(const std::vector<size_t>& active_controllers) {
    auto& controllers = this->proxy.getControllerData();

    // check for newly added controllers
    if (controllers.size() < active_controllers.size()) {
        for (const auto id : active_controllers) {
            if (controllers.find(id) == controllers.end()) {
                // register
                controllers.emplace(id, PerControllerData());
            }
        }
    }

    // check for removed controllers
    if (controllers.size() > active_controllers.size()) {
        std::vector<size_t> removable_controllers;
        // get all controllers that became absent
        for (const auto& element : controllers) {
            // check if still active controller
            if (std::find(active_controllers.begin(), active_controllers.end(), element.first) ==
                active_controllers.end()) {
                removable_controllers.push_back(element.first);
            }
        }

        // remove absent controllers
        for (const auto& id : removable_controllers) {
            std::optional<RobotIdentifier> robot = controllers.at(id).robot_id;
            // enable for localplanner if was under controll
            if (robot.has_value()) this->proxy.setLocalPlannerDisabledForRobot(robot.value(), false);
            // deregister
            controllers.erase(id);
        }
    }
}

bool RobotController::isGlobalSteering() { return this->global_movement; }

}  // namespace luhsoccer::luhviz