#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Position on defense area, based on enemy centre and other defenders
 *
 * The robot positions itself around the defense area defending the goal from a
 * cluster of enemies. A line is pulled between the  centre of the enemy cluster
 * and the goal. The line has an offset in x-y-direction if more than one robot
 * defends the penalty area. Also the point in the goal, from which the line is
 * pulled, moves in relative in y-direction corresponding to the cluster centre.
 *
 * @p related_robot Enemy0, Enemy1, Enemy2, Enemy3, Enemy4, Enemy5<br>
 * @p required_point <br>
 * @p required_double <br>
 * @p required_int DefendingRobotIndex, TotalDefendingRobots<br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class WallAtPenaltyAreaBuild : public SkillBuilder
{
   public:
    WallAtPenaltyAreaBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills