#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Rotate to a given point and drive to it, align to the heading
 *
 *
 *
 * @p related_robot <br>
 * @p required_point TargetPose<br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class goToPointAlignedBuild : public SkillBuilder
{
   public:
    goToPointAlignedBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills