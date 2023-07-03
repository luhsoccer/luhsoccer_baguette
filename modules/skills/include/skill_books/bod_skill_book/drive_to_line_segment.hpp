#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Drive to a line between two points
 *
 * First set the Dribbler to LOW, then drive to a line between p1 and p2.
 * HeadingRotationControl is ball.
 *
 * @p related_robot <br>
 * @p required_point p1, p2<br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class DriveToLineSegmentBuild : public SkillBuilder
{
   public:
    DriveToLineSegmentBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills