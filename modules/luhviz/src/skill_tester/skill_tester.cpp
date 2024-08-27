#include "skill_tester/include/skill_tester.hpp"

namespace luhsoccer::luhviz {

void SkillTester::init() {}

void SkillTester::render(bool& open) {
    if (!open) {
        return;
    }

    ImGui::PushStyleColor(ImGuiCol_Text, this->proxy.accent_text_color);
    ImGui::Begin("Manipulator", &open);
    ImGui::PopStyleColor();

    ImGui::SetCursorPos({LEFT_MARGIN, MARGIN_TOP});
    ImGui::PushFont(this->fonts.getFont(Fonts::FONT_LARGE));
    ImGui::Text("Selection");
    ImGui::PopFont();

    ImGui::BeginDisabled(this->proxy.getManipulationMode() == ManipulationMode::TELEPORT_BALL ||
                         this->proxy.getManipulationMode() == ManipulationMode::BALL_SLINGSHOT);
    static int team_index;
    renderTeamChooser(team_index);

    const bool team_ally = team_index == 0;
    const std::vector<size_t> possible_robot_ids = this->getPossibleRobotIds(team_ally, false);
    renderRobotIdChooser(team_ally, possible_robot_ids);

    static bool robot_present = true;
    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
    ImGui::Text("Present: ");
    ImGui::SameLine();
    ImGui::PushFont(this->fonts.getFont(Fonts::FONT_SMALL));
    ImGui::Checkbox("##RobotPresent", &robot_present);
    ImGui::PopFont();
    this->proxy.setRobotPresent(robot_present);

    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
    if (ImGui::Button("Teleport robot")) {
        this->proxy.getManipulationMode() = this->proxy.getManipulationMode() == ManipulationMode::SELECT
                                                ? ManipulationMode::TELEPORT_ROBOT
                                                : ManipulationMode::SELECT;
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", "Teleport the selected robot.\nShortcut: 't' ");
    ImGui::SameLine();
    if (ImGui::Button("Teleport ball")) {
        this->proxy.getManipulationMode() = this->proxy.getManipulationMode() == ManipulationMode::SELECT
                                                ? ManipulationMode::TELEPORT_BALL
                                                : ManipulationMode::SELECT;
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", "Teleport the ball.\nShortcut: 'z' ");

    ImGui::NewLine();
    ImGui::Separator();
    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
    ImGui::PushFont(this->fonts.getFont(Fonts::FONT_LARGE));
    ImGui::Text("Skill tester");
    ImGui::PopFont();

    bool disabled = !(this->proxy.getSelectedRobot().has_value() && this->proxy.getSelectedRobot().value().isAlly());
    ImGui::BeginDisabled(disabled);
    {
        renderSkillChooser(disabled);
        const std::vector<size_t> visible_related_robot_ids = this->getPossibleRobotIds(team_ally, true);
        renderSkillRequiredValuesChooser(visible_related_robot_ids);

        ImGui::NewLine();
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + LEFT_MARGIN);
        ImGui::Checkbox("Second skill", &this->proxy.getSecondSkillEnabled());
        if (this->proxy.getSecondSkillEnabled()) {
            renderSecondSkillChooser(possible_robot_ids, visible_related_robot_ids);
        } else {
            this->selected_skill2 = std::nullopt;
        }

        // clear values button
        ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
        if (ImGui::Button("Clear values")) {
            this->proxy.clearTaskData();
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Clears all Skill-related input values");

        // cancel skill button
        ImGui::SameLine();
        if (ImGui::Button("Cancel Skill(s)")) {
            if (this->proxy.getSelectedRobot().has_value()) {
                bool success = this->proxy.cancelSkills();
                if (!success) logger.warning("Skill could not be canceled");
            } else {
                logger.warning("Select a robot to cancel its current running skill");
            }
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Cancels the skill of the selected robot");

        // send skill button
        ImGui::SameLine();
        if (ImGui::Button("Send Skill(s)")) {
            bool success = this->proxy.sendSkillsToRobots();
            if (!success) logger.warning("Send Skill failed. Check if parameters are correct!");
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("Hint: Shortcut is 'x'");
        ImGui::EndDisabled();
    }
    ImGui::EndDisabled();

    // for rudimentary testing szenarios (requested by fabrice)
    renderScenarioTester();

    ImGui::End();
}

void SkillTester::renderSkillChooser(bool disabled) {
    if (first_load) {
        first_load = false;
    }

    // Skill chooser
    const std::string skill_names = this->proxy.getSkillNames();
    const float item_width_skills =
        std::max(DEFAULT_COMBO_WIDTH, Utils::getMaxItemSize(Utils::splitString(skill_names, '\0')).x);
    ImGui::PushItemWidth(item_width_skills);
    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
    ImGui::Text("Skill:");
    ImGui::SameLine();
    ImGui::SetCursorPos({TEXT_LEFT_OFFSET_POSITION, ImGui::GetCursorPosY() - 2});
    ImGui::Combo("##Skill", &selected_skill_index, skill_names.c_str());
    ImGui::PopItemWidth();
    auto selected_before = this->selected_skill;
    this->selected_skill = this->proxy.setSelectedSkill(selected_skill_index);

    if (disabled) {
        // set selected skill to nullopt
        this->selected_skill = this->proxy.setSelectedSkill(std::nullopt);
    }

    if (this->selected_skill.has_value()) {
        if (!selected_before.has_value()) {
            this->loadRelatedParamsCounts();
        } else {
            if (selected_before.value()->name.compare(this->selected_skill.value()->name) != 0) {
                // load number of requested parameters for the selected skill
                this->loadRelatedParamsCounts();
            }
        }
    }
}

void SkillTester::renderTeamChooser(int& team_index) {
    // Teamcolor chooser
    if (this->proxy.getSelectedRobot().has_value())
        team_index = this->proxy.getSelectedRobot().value().isAlly() ? 0 : 1;
    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
    ImGui::Text("Team:");
    ImGui::SameLine();
    ImGui::PushItemWidth(DEFAULT_COMBO_WIDTH);
    ImGui::SetCursorPos({TEXT_LEFT_OFFSET_POSITION, ImGui::GetCursorPosY() - 2});
    ImGui::Combo("##team_color", &team_index, this->teams.c_str());
    ImGui::PopItemWidth();
}

void SkillTester::renderRobotIdChooser(const bool& team_ally, const std::vector<size_t>& available_robot_ids) {
    // robot id chooser
    static int robot_index = 0;
    ImGui::PushItemWidth(DEFAULT_COMBO_WIDTH);
    // init selected robot index (if any selected)
    if (this->proxy.getSelectedRobot().has_value()) {
        auto index = this->getRobotIndex(this->proxy.getSelectedRobot().value(), available_robot_ids);
        if (index.has_value()) {
            robot_index = index.value();
        }
    } else if (robot_index > 1) {
        robot_index = 0;
    }
    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
    ImGui::Text("Robot Id:");
    ImGui::SameLine();
    ImGui::SetCursorPos({TEXT_LEFT_OFFSET_POSITION, ImGui::GetCursorPosY() - 2});
    ImGui::Combo("##id", &robot_index, this->listToImguiString(available_robot_ids).c_str());
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Hint: Click on robot to select");
    ImGui::PopItemWidth();

    // apply team and id to selected robot
    if (!available_robot_ids.empty() && robot_index < available_robot_ids.size()) {
        auto r = this->getRobotFromIndex(available_robot_ids[robot_index], team_ally);
        if (r.has_value()) {
            this->proxy.getSelectedRobot() = r.value();
        } else {
            this->proxy.getSelectedRobot().reset();
        }
    }

    if (this->proxy.getSelectedRobot().has_value()) {
        this->proxy.createTaskData(this->proxy.getSelectedRobot().value());
    }
}

void SkillTester::loadRelatedParamsCounts() {
    // obtain needed parameters from skills
    if (selected_skill.has_value()) {
        this->related_robots_dynamic = !selected_skill.value()->related_robot_num.has_value();
        if (!this->related_robots_dynamic) {
            num_related_robots = static_cast<int>(selected_skill.value()->related_robot_num.value());
        }
        this->required_points_dynamic = !selected_skill.value()->required_point_num.has_value();
        if (!this->required_points_dynamic) {
            num_required_points = static_cast<int>(selected_skill.value()->required_point_num.value());
        }
        this->required_bools_dynamic = !selected_skill.value()->required_bool_num.has_value();
        if (!this->required_bools_dynamic) {
            num_required_bools = static_cast<int>(selected_skill.value()->required_bool_num.value());
        }
        this->required_doubles_dynamic = !selected_skill.value()->required_double_num.has_value();
        if (!this->required_doubles_dynamic) {
            num_required_doubles = static_cast<int>(selected_skill.value()->required_double_num.value());
        }
        this->required_ints_dynamic = !selected_skill.value()->required_int_num.has_value();
        if (!this->required_ints_dynamic) {
            num_required_ints = static_cast<int>(selected_skill.value()->required_int_num.value());
        }
        this->required_strings_dynamic = !selected_skill.value()->required_string_num.has_value();
        if (!this->required_strings_dynamic) {
            num_required_strings = static_cast<int>(selected_skill.value()->required_string_num.value());
        }
    }
}

void SkillTester::renderSkillRequiredValuesChooser(const std::vector<size_t>& visible_robot_ids) {
    // related robots chooser
    if (num_related_robots > 0) {
        ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
        ImGui::Text("Related robots: %d / %d", this->proxy.getCountRelRobotsSelected(), num_related_robots);

        ImGui::PushItemWidth(TASKDATA_COMBO_OFFSET);
        static std::vector<int> related_robot_indices{};

        if (this->proxy.getSelectedRelatedRobots().empty()) {
            related_robot_indices.clear();
        }

        //

        // display related robots chooser
        for (size_t i = 0; i < static_cast<size_t>(num_related_robots); ++i) {
            displayChooseRobotItem(i, related_robot_indices);
        }
        ImGui::PopItemWidth();

        // add to task data
        if (!visible_robot_ids.empty()) {
            std::vector<std::optional<RobotIdentifier>> related_robots{};
            for (const auto& index : related_robot_indices) {
                if (static_cast<size_t>(index - 1) < this->proxy.getRobotsWithEmptyId().size() && index - 1 >= 0) {
                    auto r = this->proxy.getRobotsWithEmptyId()[index - 1];
                    related_robots.emplace_back(r);
                } else {
                    related_robots.emplace_back(std::nullopt);
                }
            }
            this->proxy.setTDRobots(related_robots);
        }
    }
    // display add item button if size is variadic (std::nullopt)
    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
    if (related_robots_dynamic && ImGui::Button("+ robot")) {
        num_related_robots++;
    }

    // required points
    if (num_required_points > 0) {
        ImGui::NewLine();
        ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
        ImGui::Text("Required points: %zu / %d", this->proxy.getTDPoints().size(), num_required_points);

        // display the already chosen points which were set with the mouse, they can be changed here but new ones
        // can only be added with clicking inside the renderView
        static std::vector<std::array<float, 3>> points{};

        if (this->proxy.getTDPoints().empty()) {
            points.clear();
        }

        for (int i = 0; i < num_required_points; ++i) {
            ImGui::SetCursorPos({LEFT_MARGIN_BIG, ImGui::GetCursorPosY() + 3});

            std::string text = "point" + std::to_string(i);
            if (this->selected_skill.has_value() && i < this->selected_skill.value()->required_point.size()) {
                text = this->selected_skill.value()->required_point[i];
            }

            ImGui::PushTextWrapPos(TASKDATA_COMBO_OFFSET - 20);
            ImGui::TextWrapped("%s", text.c_str());
            ImGui::PopTextWrapPos();
        }

        for (size_t i = 0; i < this->proxy.getTDPoints().size(); ++i) {
            ImGui::SameLine();
            ImGui::SetCursorPosX(TASKDATA_COMBO_OFFSET);
            ImGui::PushItemWidth(DEFAULT_INPUT_WIDTH);

            std::string text = "point" + std::to_string(i);
            if (this->selected_skill.has_value() && i < this->selected_skill.value()->required_point.size()) {
                text = this->selected_skill.value()->required_point[i];
            }

            const std::string text_l = "##" + text;
            if (i >= points.size()) {
                // update new points
                auto new_point = this->proxy.getTDPoints().back();
                glm::vec3 pos{new_point.x, new_point.y, Utils::rad2Degree(new_point.z)};
                points.emplace_back(std::array<float, 3>{pos.x, pos.y, pos.z});
            }
            constexpr float DRAG_SPEED = 0.1f;
            ImGui::DragFloat3(text_l.c_str(), points[i].data(), DRAG_SPEED, 0, 0, "%.2f");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Hint: Drag to change values");
            ImGui::PopItemWidth();

            ImGui::SameLine();
            std::string btn_label = "- point" + std::to_string(i);
            if (required_points_dynamic && ImGui::Button(btn_label.c_str())) {
                // remove related robot
                num_required_points--;
                points.erase(points.begin() + static_cast<long>(i));
            }
            ImGui::NewLine();
        }

        // write back the changes
        size_t i = 0;
        constexpr double EPSILON = 0.001;
        for (const auto& d : points) {
            // if points differ, update them
            const auto point = glm::dvec3{d[0], d[1], Utils::deg2Radian(d[2])};
            bool equal =
                glm::epsilonEqual(point, this->proxy.getTDPoints()[i], EPSILON) == glm::vec<3, bool>{true, true, true};

            if (!equal) {
                this->proxy.updateTDPoint(i, point);
            }
            ++i;
        }

        // TODO: points do not work
        // TODO: required values remove button und testen

        // add new Point via button
        if (this->proxy.getRemainingPointsToChoose() > 0) {
            ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
            if (ImGui::Button("Set point")) {
                this->proxy.getManipulationMode() = ManipulationMode::ADD_POINT;
            }
        }
    }
    // display add item button if size is variadic (std::nullopt)
    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
    if (required_points_dynamic && ImGui::Button("+ point")) {
        num_required_points++;
    }

    // required bools
    if (num_required_bools > 0) {
        ImGui::NewLine();
        ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
        ImGui::Text("Required bools: %d", num_required_bools);

        static std::deque<bool> bools{};
        for (int i = 0; i < num_required_bools; ++i) {
            ImGui::SetCursorPos({LEFT_MARGIN_BIG, ImGui::GetCursorPosY() + 3});

            std::string text = "bool" + std::to_string(i);
            if (this->selected_skill.has_value() && i < this->selected_skill.value()->required_bool.size()) {
                text = this->selected_skill.value()->required_bool[i];
            }

            ImGui::PushTextWrapPos(TASKDATA_COMBO_OFFSET - 20);
            ImGui::TextWrapped("%s", text.c_str());
            ImGui::PopTextWrapPos();
            ImGui::SameLine();
            ImGui::SetCursorPosX(TASKDATA_COMBO_OFFSET);
            ImGui::PushFont(this->fonts.getFont(Fonts::FONT_SMALL));
            if (i >= static_cast<int>(bools.size())) {
                bools.emplace_back(false);
            }
            const std::string text_l = "##" + text;
            ImGui::Checkbox(text_l.c_str(), &bools[i]);
            ImGui::PopFont();

            ImGui::SameLine();
            std::string btn_label = "- bool" + std::to_string(i);
            if (required_bools_dynamic && ImGui::Button(btn_label.c_str())) {
                // remove related robot
                num_required_bools--;
                bools.erase(bools.begin() + static_cast<long>(i));
            }
            ImGui::NewLine();
        }

        // add to task data
        std::vector<bool> required_bools{};
        required_bools.reserve(bools.size());
        for (const auto& b : bools) {
            required_bools.emplace_back(b);
        }
        this->proxy.setTDBools(required_bools);
    }
    // display add item button if size is variadic (std::nullopt)
    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
    if (required_bools_dynamic && ImGui::Button("+ bool")) {
        num_required_bools++;
    }

    // required ints
    if (num_required_ints > 0) {
        ImGui::NewLine();
        ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
        ImGui::Text("Required ints: %d", num_required_ints);

        static std::vector<int> ints{};
        for (int i = 0; i < num_required_ints; ++i) {
            ImGui::SetCursorPos({LEFT_MARGIN_BIG, ImGui::GetCursorPosY() + 3});

            std::string text = "int" + std::to_string(i);
            if (this->selected_skill.has_value() && i < this->selected_skill.value()->required_int.size()) {
                text = this->selected_skill.value()->required_int[i];
            }

            ImGui::PushTextWrapPos(TASKDATA_COMBO_OFFSET - 20);
            ImGui::TextWrapped("%s", text.c_str());
            ImGui::PopTextWrapPos();
            ImGui::SameLine();
            ImGui::SetCursorPosX(TASKDATA_COMBO_OFFSET);
            ImGui::PushItemWidth(DEFAULT_INPUT_WIDTH_SM);
            if (i >= static_cast<int>(ints.size())) {
                ints.emplace_back(0);
            }
            const std::string text_l = "##" + text;
            ImGui::DragInt(text_l.c_str(), &ints[i]);
            ImGui::PopItemWidth();

            ImGui::SameLine();
            std::string btn_label = "- int" + std::to_string(i);
            if (required_ints_dynamic && ImGui::Button(btn_label.c_str())) {
                // remove related robot
                num_required_ints--;
                ints.erase(ints.begin() + static_cast<long>(i));
            }
            ImGui::NewLine();
        }

        // add to task data
        std::vector<int> required_ints{};
        required_ints.reserve(ints.size());
        for (const auto& i : ints) {
            required_ints.emplace_back(i);
        }
        this->proxy.setTDInts(required_ints);
    }
    // display add item button if size is variadic (std::nullopt)
    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
    if (required_ints_dynamic && ImGui::Button("+ int")) {
        num_required_ints++;
    }

    // required doubles
    if (num_required_doubles > 0) {
        ImGui::NewLine();
        ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
        ImGui::Text("Required doubles: %d", num_required_doubles);

        static std::vector<float> doubles{0.0f};
        for (int i = 0; i < num_required_doubles; ++i) {
            ImGui::SetCursorPos({LEFT_MARGIN_BIG, ImGui::GetCursorPosY() + 3});

            std::string text = "double" + std::to_string(i);
            if (this->selected_skill.has_value() && i < this->selected_skill.value()->required_double.size()) {
                text = this->selected_skill.value()->required_double[i];
            }

            ImGui::PushTextWrapPos(TASKDATA_COMBO_OFFSET - 20);
            ImGui::TextWrapped("%s", text.c_str());
            ImGui::PopTextWrapPos();
            ImGui::SameLine();
            ImGui::SetCursorPosX(TASKDATA_COMBO_OFFSET);
            ImGui::PushItemWidth(DEFAULT_INPUT_WIDTH_SM);
            const std::string text_l = "##" + text;
            if (i >= static_cast<int>(doubles.size())) {
                doubles.emplace_back(0);
            }
            ImGui::DragFloat(text_l.c_str(), &doubles[i]);
            ImGui::PopItemWidth();

            ImGui::SameLine();
            std::string btn_label = "- double" + std::to_string(i);
            if (required_doubles_dynamic && ImGui::Button(btn_label.c_str())) {
                // remove related robot
                num_required_doubles--;
                doubles.erase(doubles.begin() + static_cast<long>(i));
            }
            ImGui::NewLine();
        }

        // add to task data
        std::vector<double> required_doubles{};
        required_doubles.reserve(doubles.size());
        for (const auto& d : doubles) {
            required_doubles.emplace_back(d);
        }
        this->proxy.setTDDoubles(required_doubles);
    }
    // display add item button if size is variadic (std::nullopt)
    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
    if (required_doubles_dynamic && ImGui::Button("+ double")) {
        num_required_doubles++;
    }

    // required strings
    if (num_required_strings > 0) {
        ImGui::NewLine();
        ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
        ImGui::Text("Required strings: %d", num_required_strings);

        static std::vector<std::string> strings{};
        for (int i = 0; i < num_required_strings; ++i) {
            ImGui::SetCursorPos({LEFT_MARGIN_BIG, ImGui::GetCursorPosY() + 3});

            std::string text = "string" + std::to_string(i);
            if (this->selected_skill.has_value() && i < this->selected_skill.value()->required_string.size()) {
                text = this->selected_skill.value()->required_string[i];
            }

            ImGui::PushTextWrapPos(TASKDATA_COMBO_OFFSET - 20);
            ImGui::TextWrapped("%s", text.c_str());
            ImGui::PopTextWrapPos();
            ImGui::SameLine();
            ImGui::SetCursorPosX(TASKDATA_COMBO_OFFSET);
            ImGui::PushItemWidth(DEFAULT_INPUT_WIDTH_SM);
            const std::string text_l = "##" + text;
            if (i >= static_cast<int>(strings.size())) {
                strings.emplace_back("");
            }
            ImGui::InputText(text_l.c_str(), &strings[i]);
            ImGui::PopItemWidth();

            ImGui::SameLine();
            std::string btn_label = "- string" + std::to_string(i);
            if (required_strings_dynamic && ImGui::Button(btn_label.c_str())) {
                // remove related robot
                num_required_strings--;
                strings.erase(strings.begin() + static_cast<long>(i));
            }
            ImGui::NewLine();
        }

        // add to task data
        std::vector<std::string> required_strings{};
        required_strings.reserve(strings.size());
        for (const auto& s : strings) {
            required_strings.emplace_back(s);
        }
        this->proxy.setTDStrings(required_strings);
    }
    // display add item button if size is variadic (std::nullopt)
    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
    if (required_strings_dynamic && ImGui::Button("+ string")) {
        num_required_strings++;
    }
}

void SkillTester::displayChooseRobotItem(size_t i, std::vector<int>& related_robot_indices) {
    if (i >= related_robot_indices.size()) {
        related_robot_indices.emplace_back(0);
    }

    const std::string i_str = std::to_string(i);

    if (i < this->proxy.getSelectedRelatedRobots().size()) {
        std::optional<RobotIdentifier> selected_robot = this->proxy.getSelectedRelatedRobots()[i];
        if (selected_robot.has_value()) {
            // get index of selected robot
            std::optional<int> index = this->getRobotIndex(selected_robot.value(), this->proxy.getRobotsWithEmptyId());
            if (index.has_value()) {
                related_robot_indices[i] = index.value();
            }
        } else {
            related_robot_indices[i] = 0;
        }
    }

    ImGui::SetCursorPos({LEFT_MARGIN_BIG, ImGui::GetCursorPosY() + 3});
    std::string title = "";
    if (this->selected_skill.has_value()) {
        if (i < this->selected_skill.value()->related_robot.size()) {
            title = this->selected_skill.value()->related_robot[i] + ":";
        } else {
            title = "robot" + i_str;
        }
    }
    ImGui::PushTextWrapPos(TASKDATA_COMBO_OFFSET - 20);
    ImGui::TextWrapped("%s", title.c_str());
    ImGui::PopTextWrapPos();
    ImGui::SameLine();
    ImGui::SetCursorPosX(TASKDATA_COMBO_OFFSET);
    const std::string label = "##" + title + i_str;
    ImGui::Combo(label.c_str(), &related_robot_indices[i], this->getAllRobotIdsWithTeam().c_str());
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Hint: STRG+Click on robot to select");
    ImGui::SameLine();
    std::string btn_label = "- robot" + i_str;
    if (related_robots_dynamic && ImGui::Button(btn_label.c_str())) {
        // remove related robot
        num_related_robots--;
        related_robot_indices.erase(related_robot_indices.begin() + static_cast<long>(i));
    }
    ImGui::NewLine();
}

void SkillTester::renderScenarioTester() {
    // Szenario chooser
    ImGui::NewLine();
    ImGui::Separator();
    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
    ImGui::PushFont(this->fonts.getFont(Fonts::FONT_LARGE));
    ImGui::Text("Scenario tester");
    ImGui::PopFont();
    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});

    static int selected_scenario_index = 0;
    std::string items = this->proxy.getAvailableScenariosAsString();
    ImGui::Combo("##Scenarios", &selected_scenario_index, items.c_str());

    ImGui::SetCursorPos({LEFT_MARGIN + 3, ImGui::GetCursorPosY() + 3});
    static int repetitions = 1;
    ImGui::Text("Repetitions: ");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(60);
    ImGui::DragInt("##repetitions: ", &repetitions, 0.1, 1, 1000);

    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
    if (ImGui::Button("Run scenario")) {
        this->proxy.executeScenario(selected_scenario_index, repetitions);
    }
}

void SkillTester::renderSecondSkillChooser(const std::vector<size_t>& available_robot_ids,
                                           const std::vector<size_t>& available_related_robot_ids) {
    // render Team and id chooser
    // -------------- render Teamcolor chooser --------------
    using namespace std::string_literals;
    const std::string teams{"ally\0enemy\0\0"s};
    static int team_index2 = 0;
    if (this->proxy.getSelectedRobot2().has_value())
        team_index2 = this->proxy.getSelectedRobot2().value().isAlly() ? 0 : 1;
    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
    ImGui::Text("Team:");
    ImGui::SameLine();
    ImGui::PushItemWidth(DEFAULT_COMBO_WIDTH);
    ImGui::SetCursorPos({TEXT_LEFT_OFFSET_POSITION, ImGui::GetCursorPosY() - 2});
    ImGui::Combo("##team_color2", &team_index2, teams.c_str());
    ImGui::PopItemWidth();

    const bool team_ally = team_index2 == 0;

    // -------------- render robot id chooser --------------
    static int robot_index2 = 0;
    ImGui::PushItemWidth(DEFAULT_COMBO_WIDTH);
    // init selected robot index (if any selected)
    if (this->proxy.getSelectedRobot2().has_value()) {
        auto index = this->getRobotIndex(this->proxy.getSelectedRobot2().value(), available_robot_ids);
        if (index.has_value()) {
            robot_index2 = index.value();
        }
    } else if (robot_index2 > 1) {
        robot_index2 = 0;
    }
    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
    ImGui::Text("Robot Id:");
    ImGui::SameLine();
    ImGui::SetCursorPos({TEXT_LEFT_OFFSET_POSITION, ImGui::GetCursorPosY() - 2});
    ImGui::Combo("##id2", &robot_index2, this->listToImguiString(available_robot_ids).c_str());
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Hint: Shift+Click on robot to select");
    ImGui::PopItemWidth();

    // apply team and id to selected robot
    if (!available_robot_ids.empty() && robot_index2 < available_robot_ids.size()) {
        auto r = this->getRobotFromIndex(available_robot_ids[robot_index2], team_ally);
        if (r.has_value()) {
            this->proxy.getSelectedRobot2() = r.value();
        } else {
            this->proxy.getSelectedRobot2().reset();
        }
    }

    if (this->proxy.getSelectedRobot2().has_value()) {
        this->proxy.createTaskData2(this->proxy.getSelectedRobot2().value());
    }

    // -------------- render skill-chooser --------------
    const std::string skill_names = this->proxy.getSkillNames();
    static int selected_skill2_index = 0;
    const float item_width_skills =
        std::max(DEFAULT_COMBO_WIDTH, Utils::getMaxItemSize(Utils::splitString(skill_names, '\0')).x);
    ImGui::PushItemWidth(item_width_skills);
    ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
    ImGui::Text("Skill:");
    ImGui::SameLine();
    ImGui::SetCursorPos({TEXT_LEFT_OFFSET_POSITION, ImGui::GetCursorPosY() - 2});
    ImGui::Combo("##Skill2", &selected_skill2_index, skill_names.c_str());
    ImGui::PopItemWidth();
    this->selected_skill2 = this->proxy.setSelectedSkill2(selected_skill2_index);

    // -------------- render required values chooser --------------
    // obtain needed parameters from skills
    int num_related_robots = 0;
    int num_required_points = 0;
    int num_required_bools = 0;
    int num_required_doubles = 0;
    int num_required_ints = 0;
    int num_required_strings = 0;
    if (selected_skill2.has_value()) {
        if (selected_skill2.value()->related_robot_num.has_value()) {
            num_related_robots = static_cast<int>(selected_skill2.value()->related_robot_num.value());
        }
        if (selected_skill2.value()->required_point_num.has_value()) {
            num_required_points = static_cast<int>(selected_skill2.value()->required_point_num.value());
        }

        if (selected_skill2.value()->required_bool_num.has_value()) {
            num_required_bools = static_cast<int>(selected_skill2.value()->required_bool_num.value());
        }

        if (selected_skill2.value()->required_double_num.has_value()) {
            num_required_doubles = static_cast<int>(selected_skill2.value()->required_double_num.value());
        }

        if (selected_skill2.value()->required_int_num.has_value()) {
            num_required_ints = static_cast<int>(selected_skill2.value()->required_int_num.value());
        }

        if (selected_skill2.value()->required_string_num.has_value()) {
            num_required_strings = static_cast<int>(selected_skill2.value()->required_string_num.value());
        }
    }

    // related robots chooser
    if (num_related_robots > 0) {
        ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
        ImGui::Text("Related robots: %d / %d", this->proxy.getCountRelRobotsSelected2(), num_related_robots);

        ImGui::PushItemWidth(TASKDATA_COMBO_OFFSET);
        static std::vector<int> related_robot_indices2{};

        if (this->proxy.getSelectedRelatedRobots2().empty()) {
            related_robot_indices2.clear();
        }

        // display related robots chooser
        for (size_t i = 0; i < static_cast<size_t>(num_related_robots); ++i) {
            if (i >= related_robot_indices2.size()) {
                related_robot_indices2.emplace_back(0);
            }

            if (i < this->proxy.getSelectedRelatedRobots2().size()) {
                auto selected_robot = this->proxy.getSelectedRelatedRobots2()[i];
                if (selected_robot.has_value()) {
                    // get index of selected robot
                    std::optional<int> index =
                        this->getRobotIndex(selected_robot.value(), this->proxy.getRobotsWithEmptyId());
                    if (index.has_value()) {
                        related_robot_indices2[i] = index.value();
                    }
                } else {
                    related_robot_indices2[i] = 0;
                }
            }

            ImGui::SetCursorPos({LEFT_MARGIN_BIG, ImGui::GetCursorPosY() + 3});

            std::string title = "title";
            if (this->selected_skill2.has_value() && i < this->selected_skill2.value()->related_robot.size()) {
                title = this->selected_skill2.value()->related_robot[i] + ":";
            }

            ImGui::PushTextWrapPos(TASKDATA_COMBO_OFFSET - 20);
            ImGui::TextWrapped("%s", title.c_str());
            ImGui::PopTextWrapPos();
            ImGui::SameLine();
            ImGui::SetCursorPosX(TASKDATA_COMBO_OFFSET);
            const std::string label = "##2" + title;
            ImGui::Combo(label.c_str(), &related_robot_indices2[i], this->getAllRobotIdsWithTeam().c_str());
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Hint: STRG+Shift+Click on robot to select");
        }
        ImGui::PopItemWidth();

        // add to task data
        if (!available_related_robot_ids.empty()) {
            std::vector<std::optional<RobotIdentifier>> related_robots{};
            for (const auto& index : related_robot_indices2) {
                if (static_cast<size_t>(index - 1) < this->proxy.getRobotsWithEmptyId().size() && index - 1 >= 0) {
                    auto r = this->proxy.getRobotsWithEmptyId()[index - 1];
                    related_robots.emplace_back(r);
                } else {
                    related_robots.emplace_back(std::nullopt);
                }
            }
            this->proxy.setTDRobots2(related_robots);
        }
    }

    // required points
    if (num_required_points > 0) {
        ImGui::NewLine();
        ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
        ImGui::Text("Required points: %zu / %d", this->proxy.getTDPoints2().size(), num_required_points);

        // display the already chosen points which were set with the mouse, they can be changed here but new ones
        // can only be added with clicking inside the renderView
        static std::vector<std::array<float, 3>> points2{};

        if (this->proxy.getTDPoints2().empty()) {
            points2.clear();
        }

        for (int i = 0; i < num_required_points; ++i) {
            ImGui::SetCursorPos({LEFT_MARGIN_BIG, ImGui::GetCursorPosY() + 3});

            std::string text = "";
            if (this->selected_skill2.has_value() && i < this->selected_skill2.value()->required_point.size()) {
                text = this->selected_skill2.value()->required_point[i];
            }

            ImGui::PushTextWrapPos(TASKDATA_COMBO_OFFSET - 20);
            ImGui::TextWrapped("%s", text.c_str());
            ImGui::PopTextWrapPos();
        }

        for (size_t i = 0; i < this->proxy.getTDPoints2().size(); ++i) {
            ImGui::SameLine();
            ImGui::SetCursorPosX(TASKDATA_COMBO_OFFSET);
            ImGui::PushItemWidth(DEFAULT_INPUT_WIDTH);
            std::string text = "";
            if (this->selected_skill2.has_value() && i < this->selected_skill2.value()->required_point.size()) {
                text = this->selected_skill2.value()->required_point[i];
            }
            const std::string text_l = "##2" + text;
            if (i >= points2.size()) {
                // update new points
                auto new_point = this->proxy.getTDPoints2().back();
                glm::vec3 pos{new_point.x, new_point.y, Utils::rad2Degree(new_point.z)};
                points2.emplace_back(std::array<float, 3>{pos.x, pos.y, pos.z});
            }
            constexpr float DRAG_SPEED = 0.1f;
            ImGui::DragFloat3(text_l.c_str(), points2[i].data(), DRAG_SPEED, 0, 0, "%.2f");
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Hint: Drag to change values");
            ImGui::PopItemWidth();
        }

        // write back the changes
        size_t i = 0;
        constexpr double EPSILON = 0.001;
        for (const auto& d : points2) {
            // if points differ, update them
            const auto point = glm::dvec3{d[0], d[1], Utils::deg2Radian(d[2])};
            bool equal =
                glm::epsilonEqual(point, this->proxy.getTDPoints2()[i], EPSILON) == glm::vec<3, bool>{true, true, true};

            if (!equal) {
                this->proxy.updateTDPoint2(i, point);
            }
            ++i;
        }

        // add new Point via button
        if (this->proxy.getRemainingPointsToChoose2() > 0) {
            ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
            if (ImGui::Button("Set point")) {
                this->proxy.getManipulationMode() = ManipulationMode::ADD_POINT;
            }
        }
    }

    // required bools
    if (num_required_bools > 0) {
        ImGui::NewLine();
        ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
        ImGui::Text("Required bools: %d", num_required_bools);

        static std::deque<bool> bools2{};
        for (int i = 0; i < num_required_bools; ++i) {
            ImGui::SetCursorPos({LEFT_MARGIN_BIG, ImGui::GetCursorPosY() + 3});

            std::string text = "";
            if (this->selected_skill2.has_value() && i < this->selected_skill2.value()->required_bool.size()) {
                text = this->selected_skill2.value()->required_bool[i];
            }

            ImGui::PushTextWrapPos(TASKDATA_COMBO_OFFSET - 20);
            ImGui::TextWrapped("%s", text.c_str());
            ImGui::PopTextWrapPos();
            ImGui::SameLine();
            ImGui::SetCursorPosX(TASKDATA_COMBO_OFFSET);
            ImGui::PushFont(this->fonts.getFont(Fonts::FONT_SMALL));
            if (i >= static_cast<int>(bools2.size())) {
                bools2.emplace_back(false);
            }
            const std::string text_l = "##2" + text;
            ImGui::Checkbox(text_l.c_str(), &bools2[i]);
            ImGui::PopFont();
        }

        // add to task data
        std::vector<bool> required_bools{};
        required_bools.reserve(bools2.size());
        for (const auto& b : bools2) {
            required_bools.emplace_back(b);
        }
        this->proxy.setTDBools2(required_bools);
    }

    // required ints
    if (num_required_ints > 0) {
        ImGui::NewLine();
        ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
        ImGui::Text("Required ints: %d", num_required_ints);

        static std::vector<int> ints2{};
        for (int i = 0; i < num_required_ints; ++i) {
            ImGui::SetCursorPos({LEFT_MARGIN_BIG, ImGui::GetCursorPosY() + 3});

            std::string text = "";
            if (this->selected_skill2.has_value() && i < this->selected_skill2.value()->required_int.size()) {
                text = this->selected_skill2.value()->required_int[i];
            }

            ImGui::PushTextWrapPos(TASKDATA_COMBO_OFFSET - 20);
            ImGui::TextWrapped("%s", text.c_str());
            ImGui::PopTextWrapPos();
            ImGui::SameLine();
            ImGui::SetCursorPosX(TASKDATA_COMBO_OFFSET);
            ImGui::PushItemWidth(DEFAULT_INPUT_WIDTH_SM);
            if (i >= static_cast<int>(ints2.size())) {
                ints2.emplace_back(0);
            }
            const std::string text_l = "##2" + text;
            ImGui::DragInt(text_l.c_str(), &ints2[i]);
            ImGui::PopItemWidth();
        }

        // add to task data
        std::vector<int> required_ints{};
        required_ints.reserve(ints2.size());
        for (const auto& i : ints2) {
            required_ints.emplace_back(i);
        }
        this->proxy.setTDInts2(required_ints);
    }

    // required doubles
    if (num_required_doubles > 0) {
        ImGui::NewLine();
        ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
        ImGui::Text("Required doubles: %d", num_required_doubles);

        static std::vector<float> doubles2{0.0f};
        for (int i = 0; i < num_required_doubles; ++i) {
            ImGui::SetCursorPos({LEFT_MARGIN_BIG, ImGui::GetCursorPosY() + 3});

            std::string text = "";
            if (this->selected_skill2.has_value() && i < this->selected_skill2.value()->required_double.size()) {
                text = this->selected_skill2.value()->required_double[i];
            }

            ImGui::PushTextWrapPos(TASKDATA_COMBO_OFFSET - 20);
            ImGui::TextWrapped("%s", text.c_str());
            ImGui::PopTextWrapPos();
            ImGui::SameLine();
            ImGui::SetCursorPosX(TASKDATA_COMBO_OFFSET);
            ImGui::PushItemWidth(DEFAULT_INPUT_WIDTH_SM);
            const std::string text_l = "##2" + text;
            if (i >= static_cast<int>(doubles2.size())) {
                doubles2.emplace_back(0);
            }
            ImGui::DragFloat(text_l.c_str(), &doubles2[i]);
            ImGui::PopItemWidth();
        }

        // add to task data
        std::vector<double> required_doubles{};
        required_doubles.reserve(doubles2.size());
        for (const auto& d : doubles2) {
            required_doubles.emplace_back(d);
        }
        this->proxy.setTDDoubles2(required_doubles);
    }

    // required strings
    if (num_required_strings > 0) {
        ImGui::NewLine();
        ImGui::SetCursorPos({LEFT_MARGIN, ImGui::GetCursorPosY() + 3});
        ImGui::Text("Required strings: %d", num_required_strings);

        static std::vector<std::string> strings2{};
        for (int i = 0; i < num_required_strings; ++i) {
            ImGui::SetCursorPos({LEFT_MARGIN_BIG, ImGui::GetCursorPosY() + 3});

            std::string text = "";
            if (this->selected_skill2.has_value() && i < this->selected_skill2.value()->required_string.size()) {
                text = this->selected_skill2.value()->required_string[i];
            }

            ImGui::PushTextWrapPos(TASKDATA_COMBO_OFFSET - 20);
            ImGui::TextWrapped("%s", text.c_str());
            ImGui::PopTextWrapPos();
            ImGui::SameLine();
            ImGui::SetCursorPosX(TASKDATA_COMBO_OFFSET);
            ImGui::PushItemWidth(DEFAULT_INPUT_WIDTH_SM);
            const std::string text_l = "##2" + text;
            if (i >= static_cast<int>(strings2.size())) {
                strings2.emplace_back("");
            }
            ImGui::InputText(text_l.c_str(), &strings2[i]);
            ImGui::PopItemWidth();
        }

        // add to task data
        std::vector<std::string> required_strings{};
        required_strings.reserve(strings2.size());
        for (const auto& s : strings2) {
            required_strings.emplace_back(s);
        }
        this->proxy.setTDStrings2(required_strings);
    }
}

}  // namespace luhsoccer::luhviz