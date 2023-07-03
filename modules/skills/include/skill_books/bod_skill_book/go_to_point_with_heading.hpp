#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Drive to TranslationalPoint and set heading to HeadingPoint
 *
 *
 *
 * @p related_robot <br>
 * @p required_point TranslationalPoint, HeadingPoint<br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class GoToPointWithHeadingBuild : public SkillBuilder
{
   public:
    GoToPointWithHeadingBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills