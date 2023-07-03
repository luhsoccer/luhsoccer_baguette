#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Kicks the ball through the target point with the given velocity
 *
 * KickVelocity is in m/s, this skill drives directly to the ball, without
 * prepositioning
 *
 * @p related_robot <br>
 * @p required_point TargetPoint<br>
 * @p required_double KickVelocity<br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class KickBallThroughTargetBuild : public SkillBuilder
{
   public:
    KickBallThroughTargetBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills