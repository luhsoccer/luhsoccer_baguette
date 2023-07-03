#pragma once

#include "local_planner/skills/skill.hpp"
#include <map>

namespace luhsoccer::skills {

template <typename E>
class SkillBook {
   public:
    friend class SkillBuilder;

    const local_planner::Skill& getSkill(E skill_name) const { return this->skills.at(skill_name); }

    static bool taskValid(const local_planner::Skill& skill, const local_planner::TaskData& data) {
        return skill.taskDataValid(data);
    }

    bool taskValid(const E& skill_name, const local_planner::TaskData& task) {
        return SkillBook::taskValid(this->getSkill(skill_name), task);
    }

    std::vector<std::pair<std::string, E>> getSkillList() const {
        std::vector<std::pair<std::string, E>> names{};

        names.reserve(this->skills.size());

        for (const auto& [enumerator, skill] : this->skills) {
            names.push_back({skill.name, enumerator});
        }
        return names;
    }

   protected:
    explicit SkillBook(std::map<E, local_planner::Skill> skills) : skills(std::move(skills)){};
    std::map<E, local_planner::Skill> skills;
};

}  // namespace luhsoccer::skills