#pragma once

#include "robot_control/skills/skill.hpp"
namespace luhsoccer::skills {

template <typename E>
class SkillBook {
   public:
    friend class SkillBuilder;

    const robot_control::Skill& getSkill(E skill_name) const { return this->skills.at(skill_name); }

    static bool taskValid(const robot_control::Skill& skill, const robot_control::TaskData& data) {
        return skill.taskDataValid(data);
    }

    bool taskValid(const E& skill_name, const robot_control::TaskData& task) {
        return SkillBook::taskValid(this->getSkill(skill_name), task);
    }

    std::vector<std::pair<std::string, E>> getSkillList() const {
        std::vector<std::pair<std::string, E>> names{};

        names.reserve(this->skills.size());

        for (const auto& [enumerator, skill] : this->skills) {
            names.push_back({skill.name, enumerator});
        }

        std::sort(names.begin(), names.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

        return names;
    }

   protected:
    explicit SkillBook(std::map<E, robot_control::Skill> skills) : skills(std::move(skills)){};
    std::map<E, robot_control::Skill> skills;
};

}  // namespace luhsoccer::skills