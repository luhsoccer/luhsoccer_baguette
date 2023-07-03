#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Drive on a between the enemy robot and the ball, looking to the ball
 *
 * The skill is endless, at first it turns off the dribbler, then there is a
 * DriveStep with two target features, a circle around the enemy robot and a
 * line between the enemy robot and the ball. The robot looks to the ball. The
 * radius of the circle is defined by a config parameter.
 *
 * @p related_robot EnemyRobot<br>
 * @p required_point <br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class MarkEnemyToBallBuild : public SkillBuilder
{
   public:
    MarkEnemyToBallBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills