#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Block the goalies los and kick the ball back
 *
 * This skill is endless. Theres one DriveStep that has two target features, a
 * circle around the enemys goal and a line through the ball and the enemy. The
 * kicker kicks when the ball is in dribbler. The dribbler is high. The
 * related_robot needs to be the enemy goalie.
 *
 * @p related_robot EnemyBallCarrier<br>
 * @p required_point <br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class BlockGoalieLoSBuild : public SkillBuilder
{
   public:
    BlockGoalieLoSBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills