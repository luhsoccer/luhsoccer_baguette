#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Drive in the ball to kick a bit
 *
 * Kicks the ball in the direction of target by driving in the ball with
 * max_velocity.
 *
 * @p related_robot <br>
 * @p required_point target<br>
 * @p required_double max_velocity<br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class DriveInBallBuild : public SkillBuilder
{
   public:
    DriveInBallBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills