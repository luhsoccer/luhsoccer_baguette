#pragma once

#include "skill_books/bod_skill_book.hpp"
#include "skill_books/game_skill_book.hpp"
#include "skill_books/test_skill_book.hpp"
#include <variant>

namespace luhsoccer::skills {

using SkillNames = std::variant<BodSkillNames, GameSkillNames, TestSkillNames>;

class SkillLibrary {
   public:
    SkillLibrary();

    [[nodiscard]] const robot_control::Skill& getSkill(const SkillNames& skill_name) const;

    bool taskValid(const SkillNames& skill_name, const robot_control::TaskData& task) {
        return this->getSkill(skill_name).taskDataValid(task);
    }
    [[nodiscard]] std::vector<std::pair<std::string, SkillNames>> getSkillList() const;

    BodSkillBook bod_book;
    GameSkillBook game_book;
    TestSkillBook test_book;
};

}  // namespace luhsoccer::skills