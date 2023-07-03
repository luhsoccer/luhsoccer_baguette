#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Intercepts the ball
 *
 * At first the skill waits until the ball starts moving. While waiting the
 * robot turns to the ball. When it starts moving, the robot drives to the line
 * through the ball position and a past ball position.
 *
 * @p related_robot <br>
 * @p required_point <br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class ReceiveBallBuild : public SkillBuilder
{
   public:
    ReceiveBallBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills