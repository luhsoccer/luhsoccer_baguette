#include "skill_books/test_skill_book.hpp"

#include "config_provider/config_store_main.hpp"

// include skill file here

// end of includes DO NOT MODIFY THIS LINE

namespace luhsoccer::skills {

TestSkillBook::TestSkillBook(const config_provider::ConfigStore& cs)
    : SkillBook<TestSkillNames>({
          // add skills after this line (do not delete this line)

          // do not delete this line either
      }){};

}
