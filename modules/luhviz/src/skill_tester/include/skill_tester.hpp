#pragma once

#include <array>
#include "imgui.h"
#include "logger/logger.hpp"
#include "include/data_proxy.hpp"
#include "robot_identifier.hpp"
#include <imgui_internal.h>
#include "imgui_backend/imgui_stdlib.h"
#include "common/include/utils.hpp"
#include "common/include/fonts.hpp"

namespace luhsoccer::luhviz {

using namespace std::string_literals;
class SkillTester {
   public:
    SkillTester(DataProxy& proxy, Fonts& fonts) : proxy(proxy), fonts(fonts){};
    // ----- members -----

    // ----- methods -----
    void init();
    void render();

   private:
    static constexpr int MARGIN_TOP = 28;
    static constexpr int LEFT_MARGIN = 10;
    static constexpr int LEFT_MARGIN_BIG = 30;
    static constexpr int TEXT_LEFT_OFFSET_POSITION = 80;
    static constexpr int TEXT_LEFT_OFFSET_POSITION_BIG = 110;
    static constexpr float TASKDATA_COMBO_OFFSET = 150;
    static constexpr float DEFAULT_COMBO_WIDTH = 140;
    static constexpr float DEFAULT_INPUT_WIDTH = 200;
    static constexpr float DEFAULT_INPUT_WIDTH_SM = 120;

    // ----- members -----
    logger::Logger logger{"luhviz/skill_tester"};
    DataProxy& proxy;
    Fonts& fonts;
    std::optional<local_planner::Skill*> selected_skill;
    std::optional<local_planner::Skill*> selected_skill2;

    const std::string teams{"ally\0enemy\0\0"s};

    // ----- methods -----
    /**
     * @brief displays the skill chooser Dropdown
     *
     */
    void renderSkillChooser(bool disabled);

    /**
     * @brief displays the team chooser dropdown
     *
     * @param team_index index of the current team (0 = ally, 1 = enemy)
     */
    void renderTeamChooser(int& team_index);

    /**
     * @brief displays the robot id chooser dropdown
     *
     * @param team_ally true if ally team is chosen
     * @param available_robot_ids the available ids of the robots on the field
     */
    void renderRobotIdChooser(const bool& team_ally, const std::vector<size_t>& available_robot_ids);

    /**
     * @brief displays the required values ui elements (required points, required bools, ...)
     *
     * @param available_robot_ids the available ids of the robots on the field
     */
    void renderSkillRequiredValuesChooser(const std::vector<size_t>& available_robot_ids);

    /**
     * @brief this is currently just showing a button to start 1 hardcoded szenario (requested for testing by fabrice)
     *
     */
    void renderScenarioTester();

    /**
     * @brief displays the optional second skillchooser to send 2 skills at the same time
     *
     */
    void renderSecondSkillChooser(const std::vector<size_t>& available_robot_ids,
                                  const std::vector<size_t>& available_related_robot_ids);

    /**
     * @brief Get all available Robot Ids
     *
     * @param ally
     * @return std::vector<size_t>
     */
    std::vector<size_t> getPossibleRobotIds(bool ally, bool add_empty_id) {
        // the first element acts as "no robot selected"
        std::vector<size_t> ids{std::numeric_limits<size_t>::max()};
        if (add_empty_id) {
            ids.emplace_back(std::numeric_limits<size_t>::max());
        }
        // get all available skills and save them
        for (const auto& robot : this->proxy.getPossibleRobots()) {
            if (robot.isAlly() == ally) {
                ids.emplace_back(robot.id);
            }
        }
        return ids;
    }

    /**
     * @brief Get all available Robot Ids
     *
     * @param ally
     * @return std::vector<size_t>
     */
    std::vector<size_t> getVisibleRobotIds(bool ally, bool add_empty_id) {
        // the first element acts as "no robot selected"
        std::vector<size_t> ids{std::numeric_limits<size_t>::max()};
        if (add_empty_id) {
            ids.emplace_back(std::numeric_limits<size_t>::max());
        }
        // get all available skills and save them
        for (const auto& robot : this->proxy.getVisibleRobots()) {
            if (robot.isAlly() == ally) {
                ids.emplace_back(robot.id);
            }
        }
        return ids;
    }

    // TODO: id: possible robots und related: visible robots sauber trennen!

    /**
     * @brief Get the All Robot Ids with the team info
     *
     * @return std::string
     */
    std::string getAllRobotIdsWithTeam() {
        using namespace std::string_literals;

        std::string list_values{"-\0Empty_Identifier\0"s};
        // get all available skills and save them
        for (const auto& robot : this->proxy.getPossibleRobots()) {
            std::string id = std::to_string(robot.id);
            std::string team = robot.team == Team::ALLY ? "Ally" : "Enemy";
            auto team_str = " (" + team + ")";
            list_values += id + team_str + "\0"s;
        }
        list_values += "\0"s;
        return list_values;
    }

    /**
     * @brief converts a list to a string with elements separated with null terminators.
     *  this is needed for the ImGui:Combo box selectors
     *
     * @param list
     * @return std::string
     */
    [[nodiscard]] std::string listToImguiString(const std::vector<size_t>& list) const {
        using namespace std::string_literals;

        std::string list_values{"-\0"s};
        // get all available skills and save them
        for (const auto& value : list) {
            if (value < std::numeric_limits<size_t>::max()) {
                list_values += std::to_string(value) + "\0"s;
            }
        }
        list_values += "\0"s;
        return list_values;
    }

    /**
     * @brief Given a robot, returns the index of the robots id in the available robot ids from the selected team
     *
     * @param robot
     * @param available_robot_ids
     * @return std::optional<int>
     */
    std::optional<int> getRobotIndex(const RobotIdentifier& robot, const std::vector<size_t>& available_robot_ids) {
        int index = 0;
        for (const auto& id : available_robot_ids) {
            if (id == robot.id) {
                return index;
            }
            ++index;
        }
        return std::nullopt;
    }

    /**
     * @brief Given a robot, returns the index of the robots id in the available robot ids from the selected team
     *
     * @param robot
     * @param available_robot_ids
     * @return std::optional<int>
     */
    std::optional<int> getRobotIndex(const RobotIdentifier& robot, const std::vector<RobotIdentifier>& robots) {
        int index = 0;
        for (const auto& r : robots) {
            if (r.id == robot.id && r.team == robot.team) {
                return index + 1;
            }
            ++index;
        }
        return std::nullopt;
    }

    /**
     * @brief given the robot id, returns the robot from the robots vector which matches the id and is in the selected
     * team
     *
     * @param robot_id
     * @param ally_selected
     * @return std::optional<RobotIdentifier>
     */
    std::optional<RobotIdentifier> getRobotFromIndex(size_t robot_id, bool team_ally) {
        for (const auto& robot : this->proxy.getPossibleRobots()) {
            if (robot.id == robot_id && robot.isAlly() == team_ally) {
                return robot;
            }
        }
        return std::nullopt;
    }
};
}  // namespace luhsoccer::luhviz