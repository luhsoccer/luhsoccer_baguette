#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Move on a circle around the ball, looking to the ball
 *
 * First the dribbler is turned off. Then the robot moves to a circle around the
 * around the ball, the radius of the circle is defined by a config parameter.
 * The robot looks to the ball.
 *
 * @p related_robot <br>
 * @p required_point <br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class MoveToBallTurnRadiusBuild : public SkillBuilder
{
   public:
    MoveToBallTurnRadiusBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills