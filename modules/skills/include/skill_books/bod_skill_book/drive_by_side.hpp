#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

class DriveBySideBuild : public SkillBuilder
{
   public:
    DriveBySideBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills