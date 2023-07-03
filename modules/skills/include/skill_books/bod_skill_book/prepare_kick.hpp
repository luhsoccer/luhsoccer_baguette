#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Prepare a kick by moving close to the ball and turning to the target
 *
 * This is done with a DriveStep and a TurnAroundBallStep. The radius is
 * hardcoded with 0.3m.
 *
 * @p related_robot <br>
 * @p required_point TargetPoint<br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class PrepareKickBuild : public SkillBuilder
{
   public:
    PrepareKickBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills