#include "luhconfig/include/luhconfig.hpp"

#define IMGUI_DEFINE_MATH_OPERATORS
#include "imgui_internal.h"

namespace luhsoccer::luhviz {

void LuhvizConfig::init() { this->reset_icon.create(this->reset_icon_path, false, true); }

void LuhvizConfig::render(bool* open) {
    constexpr ImVec2 MIN_WINDOW_SIZE{100, 200};
    constexpr ImVec2 MAX_WINDOW_SIZE{INFINITY, INFINITY};

    constexpr bool SHOW_LUHVIZ_INTERNAL = false;

    // only render if its opened and discard changes
    if (!*open) {
        return;
    }

    ImGui::SetNextWindowSizeConstraints(MIN_WINDOW_SIZE, MAX_WINDOW_SIZE);
    ImGui::Begin("Luhconfig", open, ImGuiWindowFlags_HorizontalScrollbar);

    const float cursor_start = ImGui::GetCursorPosX() + 15;
    const float cursor_start_group = ImGui::GetCursorPosX() + 10;

    size_t bool_index = 0, int_index = 0, float_index = 0, string_index = 0;

    ImGui::BeginChild("Tabs", {0, 0}, false, ImGuiWindowFlags_AlwaysVerticalScrollbar);
    if (ImGui::BeginTabBar("Parameter Configuration")) {
        // Iterate over all config structs
        static std::string last_active_tab_name{""};
        static std::string last_hovered_tab_name{""};
        for (auto& cfg : this->data_proxy.getAllConfigParams()) {
            // Iterate over all paramters withing the current config struct
            std::string config_name = cfg.first;

            if (config_name.compare("luhviz_internal") == 0 && !SHOW_LUHVIZ_INTERNAL) {
                // dont render luhviz internal config values
                continue;
            }

            // default name is "Global"
            if (config_name.empty() || config_name.compare(" ") == 0) config_name = "Global";

            // display Tab for every config
            ImGui::PushStyleColor(ImGuiCol_Text, data_proxy.accent_text_color);

            bool begin_tab_item = ImGui::BeginTabItem(config_name.c_str());

            if (ImGui::IsItemHovered()) last_hovered_tab_name = config_name;

            ImGui::PopStyleColor();

            if (begin_tab_item) {
                last_active_tab_name = config_name;
                // evaluate the type of the parameter, cast it and access its value / change i
                bool any_non_default = false;
                auto& params = cfg.second;
                std::string last_group_name{""};
                ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 25);
                for (auto& param : params) {
                    std::string group = param->group;
                    if (group.length() > 0) {
                        group[0] = std::toupper(group[0]);
                        // replace config keys '_' with space for displaying the name
                        std::replace(group.begin(), group.end(), '_', ' ');

                        if (group.compare(last_group_name) != 0) {
                            // display group name
                            ImGui::SetCursorPosX(cursor_start_group);
                            ImGui::TextDisabled("%s:", group.c_str());
                            last_group_name = group;
                        }
                    }

                    // display name
                    ImGui::SetCursorPosX(cursor_start);
                    ImGui::Text("%s", param->key.c_str());

                    ImGui::SameLine();
                    const float x_offset = 50.0f;
                    ImGui::SetCursorPosX(900 / 2 - x_offset);
                    ImGui::PushItemWidth(900 / 2);

                    try {
                        switch (param->getType()) {
                            case ParamType::INT: {
                                auto& el = dynamic_cast<ConfigInt&>(*param);
                                if (int_index >= this->ints.size())
                                    this->ints.emplace_back(el.val);
                                else
                                    this->ints[int_index] = el.val;

                                ImGui::BeginDisabled(!el.is_writeable);
                                // UI for INT parameter
                                if (el.min == std::numeric_limits<int>::min() ||
                                    el.max == std::numeric_limits<int>::max()) {
                                    // input field for not ranged values
                                    ImGui::DragScalar(("##" + el.key).c_str(), ImGuiDataType_S32,
                                                      &this->ints[int_index]);
                                } else {
                                    // slider for ranged values
                                    ImU32 min = el.min;
                                    ImU32 max = el.max;
                                    ImGui::SliderScalar(("##" + el.key).c_str(), ImGuiDataType_S32,
                                                        &this->ints[int_index], &min, &max);
                                }

                                // tooltip showing the description of the parameter
                                if (ImGui::IsItemHovered()) {
                                    std::string desc =
                                        el.description + "\nDefault value: " + std::to_string(el.getDefaultValue());
                                    ImGui::SetTooltip("%s", desc.c_str());
                                }

                                ImGui::EndDisabled();

                                // auto-save value if has changed
                                if (el.is_writeable && el.val != this->ints[int_index]) {
                                    el.val = this->ints[int_index];
                                    el.synced_with_config_provider = false;
                                    this->saved = false;
                                }

                                int_index++;
                                break;
                            }
                            case ParamType::DOUBLE: {
                                auto& el = dynamic_cast<ConfigDouble&>(*param);
                                if (float_index >= this->floats.size())
                                    this->floats.emplace_back(static_cast<float>(el.val));
                                else
                                    this->floats[float_index] = static_cast<float>(el.val);

                                ImGui::BeginDisabled(!el.is_writeable);

                                // UI for FLOAT/DOUBLE parameter
                                if (el.min == std::numeric_limits<double>::min() ||
                                    el.max == std::numeric_limits<double>::max()) {
                                    // input field for not ranged values
                                    constexpr float SCROLL_SPEED = 0.0005f;
                                    ImGui::DragFloat(("##" + el.key).c_str(), &this->floats[float_index], SCROLL_SPEED);
                                } else {
                                    // slider for ranged values
                                    auto min = static_cast<float>(el.min);
                                    auto max = static_cast<float>(el.max);
                                    ImGui::SliderFloat(("##" + el.key).c_str(), &this->floats[float_index], min, max);
                                }

                                // tooltip showing the description of the parameter
                                if (ImGui::IsItemHovered()) {
                                    std::string desc =
                                        el.description + "\nDefault value: " + std::to_string(el.getDefaultValue());
                                    ImGui::SetTooltip("%s", desc.c_str());
                                }

                                ImGui::EndDisabled();

                                // auto-save value if has changed
                                if (el.is_writeable && el.val != this->floats[float_index]) {
                                    el.val = this->floats[float_index];
                                    el.synced_with_config_provider = false;
                                    this->saved = false;
                                }

                                float_index++;
                                break;
                            }
                            case ParamType::BOOL: {
                                auto& el = dynamic_cast<ConfigBool&>(*param);

                                if (bool_index >= this->bools.size()) {
                                    this->bools.emplace_back(el.val);
                                } else {
                                    this->bools[bool_index] = el.val;
                                }

                                ImGui::BeginDisabled(!el.is_writeable);

                                // UI for BOOL parameter
                                ImGui::Checkbox(("##" + el.key).c_str(), &this->bools[bool_index]);

                                // tooltip showing the description of the parameter
                                if (ImGui::IsItemHovered()) {
                                    std::string desc =
                                        el.description + "\nDefault value: " + std::to_string(el.getDefaultValue());
                                    ImGui::SetTooltip("%s", desc.c_str());
                                }

                                ImGui::EndDisabled();

                                // auto-save value if has changed
                                if (el.is_writeable && el.val != this->bools[bool_index]) {
                                    el.val = this->bools[bool_index];
                                    el.synced_with_config_provider = false;
                                    this->saved = false;
                                }

                                bool_index++;
                                break;
                            }
                            case ParamType::STRING: {
                                auto& el = dynamic_cast<ConfigString&>(*param);

                                if (string_index >= this->strings.size())
                                    this->strings.emplace_back(el.val);
                                else
                                    this->strings[string_index] = el.val;

                                ImGui::BeginDisabled(!el.is_writeable);

                                // UI for TEXT parameter
                                ImGui::InputText(("##" + el.key).c_str(), &this->strings[string_index]);

                                // tooltip showing the description of the parameter
                                if (ImGui::IsItemHovered()) {
                                    std::string desc = el.description + "\nDefault value: " + el.getDefaultValue();
                                    ImGui::SetTooltip("%s", desc.c_str());
                                }

                                ImGui::EndDisabled();

                                // auto-save value if has changed
                                if (el.is_writeable && el.val.compare(this->strings[string_index]) != 0) {
                                    el.val = this->strings[string_index];
                                    el.synced_with_config_provider = false;
                                    this->saved = false;
                                }

                                string_index++;
                                break;
                            }
                            default:
                                break;
                        }
                    } catch (std::bad_cast& e) {
                        LOG_ERROR(logger, "{}", e.what());
                    }

                    ImGui::PopItemWidth();

                    // check if value is different to default value, display reset button if difference
                    if (param->hasValueChanged()) {
                        any_non_default = true;
                        constexpr ImVec2 BUTTON_SIZE = {16, 16};
                        constexpr int OFFSET = 40;
                        ImGui::SameLine();
                        ImGui::SetCursorPosX(ImGui::GetWindowWidth() - OFFSET);
                        const std::string b_id = "##resetToDefault_" + param->key;
                        if (ImGui::ImageButton(b_id.c_str(), this->reset_icon.getImguiId(), BUTTON_SIZE)) {
                            param->reset();
                            this->data_proxy.updateConfigParam(param->key, param);
                            this->saved = false;
                        }
                    }
                }

                // button for resetting current config
                constexpr ImVec2 BUTTON_OFFSET = {100, 30};
                ImGui::SetCursorPos({ImGui::GetWindowWidth() - BUTTON_OFFSET.x, BUTTON_OFFSET.y});
                ImGui::BeginDisabled(!any_non_default);
                if (ImGui::Button("Reset config")) {
                    for (auto& p : cfg.second) {
                        p->reset();
                        this->data_proxy.updateConfigParam(p->key, p);
                        this->saved = false;
                    }
                }
                ImGui::EndDisabled();

                ImGui::EndTabItem();
            }
        }
        ImGui::EndTabBar();
    }

    const auto window_height = ImGui::GetWindowHeight();

    // buttons
    const float w = ImGui::GetWindowWidth();
    const float button_offset_x = 60;
    const float button_offset_y = 30;
    // button for saving
    ImGui::BeginDisabled(this->saved);
    ImGui::SetCursorPos({w - button_offset_x, window_height - button_offset_y});
    if (ImGui::Button("Save")) {
        this->saveConfig();
    }
    ImGui::EndDisabled();

    ImGui::EndChild();

    ImGui::End();

    // update the config values every 0.5 sec
    if (std::chrono::system_clock::now() - last_write_time > std::chrono::milliseconds(WRITE_VALUES_DELAY_MS)) {
        for (auto& cfg : this->data_proxy.getAllConfigParams()) {
            for (auto& param : cfg.second) {
                if (!param->synced_with_config_provider) {
                    const std::string key = param->key;
                    param->synced_with_config_provider = true;
                    this->data_proxy.updateConfigParam(key, param);
                }
            }
        }
        last_write_time = std::chrono::system_clock::now();
    }

    // load the values from config_provider every few seconds because there might be changes
    // from other sources like python interface
    if (std::chrono::system_clock::now() - last_update_time > std::chrono::milliseconds(UPDATE_VALUES_DELAY_MS)) {
        this->data_proxy.loadAllConfigs();
        last_update_time = std::chrono::system_clock::now();
    }
}

void LuhvizConfig::saveConfig() {
    // save values on button click
    for (auto& cfg : this->data_proxy.getAllConfigParams()) {
        for (auto& param : cfg.second) {
            if (!param->synced_with_config_provider) {
                const std::string key = param->key;
                param->synced_with_config_provider = true;
                this->data_proxy.updateConfigParam(key, param);
            }
        }
    }
    this->data_proxy.saveAllConfigs();
    this->saved = true;
    LOG_INFO(logger, "Configs saved!");
}

void LuhvizConfig::saveAtClose() {
    const auto& it = this->data_proxy.getAllConfigParams().find("luhviz_internal");
    if (it != this->data_proxy.getAllConfigParams().end()) {
        // update the config_provider values of luhviz
        for (auto& param : it->second) {
            if (!param->synced_with_config_provider) {
                const std::string key = param->key;
                param->synced_with_config_provider = true;
                this->data_proxy.updateConfigParam(key, param);
            }
        }
    }
    this->data_proxy.saveLuhvizConfig();
}

}  // namespace luhsoccer::luhviz