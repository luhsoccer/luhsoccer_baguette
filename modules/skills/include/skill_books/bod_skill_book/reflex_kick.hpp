#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Intercept the ball and kick it directly to TargetPosition
 *
 * First the robot waits for a kick. When the ball moves, the robot turns to
 * TargetPosition and sets kicker to kick when ball in dribbler. Then the robot
 * drives on the line the ball moves on.
 *
 * @p related_robot <br>
 * @p required_point TargetPosition<br>
 * @p required_double kick_velocity<br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class ReflexKickBuild : public SkillBuilder
{
   public:
    ReflexKickBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills