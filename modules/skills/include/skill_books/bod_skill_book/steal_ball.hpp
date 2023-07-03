#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Steal the ball from an enemy robot
 *
 * This skill drives to with dribbler high to the ball to get it out of an
 * enemys dribbler. The enemy robot should not move, otherwise the skill will
 * probably fail.
 *
 * @p related_robot ball_carrier<br>
 * @p required_point <br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class StealBallBuild : public SkillBuilder
{
   public:
    StealBallBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills