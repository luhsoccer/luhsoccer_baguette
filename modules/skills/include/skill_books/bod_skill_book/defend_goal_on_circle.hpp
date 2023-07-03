#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Moves on a circle around the goal to defend it.
 *
 * The circle is defined through two config parameters. The robot moves on the
 * circle until the ball is shoot in direction of the goal. Then the robot will
 * intercept the ball.
 *
 * @p related_robot <br>
 * @p required_point <br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class DefendGoalOnCircleBuild : public SkillBuilder
{
   public:
    DefendGoalOnCircleBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills