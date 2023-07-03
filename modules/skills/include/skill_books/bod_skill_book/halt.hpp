#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Turn off dribbler
 *
 *
 *
 * @p related_robot <br>
 * @p required_point <br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class HaltBuild : public SkillBuilder
{
   public:
    HaltBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills