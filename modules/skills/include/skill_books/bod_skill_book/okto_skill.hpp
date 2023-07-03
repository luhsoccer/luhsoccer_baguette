#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Moves in a given direction
 *
 * There is one infinite drive step with heading to the ball. The direction is
 * given by a vector, so the skill takes two doubles for x and y of that vector.
 *
 * @p related_robot <br>
 * @p required_point <br>
 * @p required_double v_x, v_y<br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class OktoSkillBuild : public SkillBuilder
{
   public:
    OktoSkillBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills