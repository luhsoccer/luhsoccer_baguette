#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Drive on a between the enemy robot and our goals center, looking to
 * the enemy
 *
 * The skill is endless, at first it turns off the dribbler, then there is a
 * DriveStep with two target features, a circle around the enemy robot and a
 * line between the enemy robot and goal ally center. The robot looks to the
 * enemy robot. The radius of the circle is defined by a config parameter.
 *
 * @p related_robot EnemyRobot<br>
 * @p required_point <br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class MarkEnemyToGoalBuild : public SkillBuilder
{
   public:
    MarkEnemyToGoalBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills