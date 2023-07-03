#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Moves on a line in front of the goal to defend it.
 *
 * The skill uses two target features, a line in front of the goal, and a line
 * through the ball and the given enemy ball carrier.
 *
 * @p related_robot EnemyBallCarrier<br>
 * @p required_point <br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class DefendGoallineBuild : public SkillBuilder
{
   public:
    DefendGoallineBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills