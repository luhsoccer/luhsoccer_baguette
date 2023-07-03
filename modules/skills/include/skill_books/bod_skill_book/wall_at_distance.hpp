#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Position on prependicular wall from ball to goal
 *
 * The robot positions itself on prependicular wall from ball to goal. A line is
 * pulled between the ball and the goal. The line has an offset in x-y-direction
 * if more than one robot defends the goal. Also the point in the goal, from
 * which the line is pulled, moves in relative in y-direction corresponding to
 * the ball. A second line is pulled perpendicular to the first line at the set
 * distance from the goal.
 *
 * @p related_robot <br>
 * @p required_point <br>
 * @p required_double WallDistance<br>
 * @p required_int DefendingRobotIndex, TotalDefendingRobots<br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class WallAtDistanceBuild : public SkillBuilder
{
   public:
    WallAtDistanceBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills