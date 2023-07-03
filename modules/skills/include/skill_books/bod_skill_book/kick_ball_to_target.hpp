#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Kicks the ball to the target point
 *
 * The velocity is not calculated correct, so this skill should not be used
 *
 * @p related_robot <br>
 * @p required_point TargetPoint<br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class KickBallToTargetBuild : public SkillBuilder
{
   public:
    KickBallToTargetBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills