#include "skill_books/skill_library.hpp"

#include "config_provider/config_store_main.hpp"
#include "core/visit.hpp"
namespace luhsoccer::skills {

SkillLibrary::SkillLibrary()
    : bod_book(config_provider::ConfigProvider::getConfigStore()),
      game_book(config_provider::ConfigProvider::getConfigStore()),
      test_book(config_provider::ConfigProvider::getConfigStore()){};

const robot_control::Skill& SkillLibrary::getSkill(const SkillNames& skill_name) const {
    auto visitor = overload{[this](const BodSkillNames& bod_skill) -> const robot_control::Skill& {
                                return this->bod_book.getSkill(bod_skill);
                            },
                            [this](const TestSkillNames& test_skill) -> const robot_control::Skill& {
                                return this->test_book.getSkill(test_skill);
                            },
                            [this](const GameSkillNames& game_skill) -> const robot_control::Skill& {
                                return this->game_book.getSkill(game_skill);
                            }};

    return std::visit(visitor, skill_name);
}

std::vector<std::pair<std::string, SkillNames>> SkillLibrary::getSkillList() const {
    std::vector<std::pair<std::string, SkillNames>> names;
    auto bod_names = this->bod_book.getSkillList();
    names.insert(names.end(), bod_names.begin(), bod_names.end());

    auto game_names = this->game_book.getSkillList();
    names.insert(names.end(), game_names.begin(), game_names.end());

    auto test_names = this->test_book.getSkillList();
    names.insert(names.end(), test_names.begin(), test_names.end());

    return names;
}

}  // namespace luhsoccer::skills