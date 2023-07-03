#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Passes the ball to a given robot
 *
 * These are the steps of the skill:
 * Drive close to the ball(MOVE_TO_BALL_TURN_RADIUS)
 * Turn around the ball to face the target robot
 * Set kick when ball is in dribbler
 * Drive into the ball
 *
 * Theres still a not optimized calculation for the kick velocity
 *
 * @p related_robot target_robot<br>
 * @p required_point <br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class passBallToRobotBuild : public SkillBuilder
{
   public:
    passBallToRobotBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills