#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Forwards dribbling allows the robot to dribble the ball in regular
 * play or ball placement
 *
 * The robot prepositions itself towards the target position before it retreives
 * the ball and dribbles with it forwards to the target position. If the robot
 * dribbles during ball placement or during normal play is represented through a
 * boolean input parameter. The two cases result in different finishing steps.
 * Dribbling during normal play ends with the arrival at the target point or
 * once the robot dribbled >1m. Dribbling during ball placement turns off the
 * dribbler once it arrived at the target position and moves away from the ball,
 * before ending the skill.
 *
 * @p related_robot <br>
 * @p required_point TargetPoint<br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool BallPlacement<br>
 * @p required_string <br>
 */
class ForwardsDribblingBuild : public SkillBuilder
{
   public:
    ForwardsDribblingBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills