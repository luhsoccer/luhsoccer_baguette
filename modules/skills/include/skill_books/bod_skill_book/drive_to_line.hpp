#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Moves on a line which is defined by a point and an angle.
 *
 * The angle needs to be in radians.
 *
 * @p related_robot <br>
 * @p required_point point_on_line<br>
 * @p required_double angle_line<br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class DriveToLineBuild : public SkillBuilder
{
   public:
    DriveToLineBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills