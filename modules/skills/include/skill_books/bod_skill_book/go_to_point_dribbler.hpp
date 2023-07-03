#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Turn on dribbler and drive to target_position
 *
 *
 *
 * @p related_robot <br>
 * @p required_point target_position<br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class GoToPointDribblerBuild : public SkillBuilder
{
   public:
    GoToPointDribblerBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};

}  // namespace luhsoccer::skills