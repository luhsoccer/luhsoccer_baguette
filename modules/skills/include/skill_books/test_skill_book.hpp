#pragma once

#include "skills/skill_book.hpp"

namespace luhsoccer::config_provider {
struct ConfigStore;
}

namespace luhsoccer::skills {

enum class TestSkillNames {

};  // DO NOT MODIFY THIS LINE

class TestSkillBook : public SkillBook<TestSkillNames> {
   public:
    explicit TestSkillBook(const config_provider::ConfigStore& cs);
};
}  // namespace luhsoccer::skills