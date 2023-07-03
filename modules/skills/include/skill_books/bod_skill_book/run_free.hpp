#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Run free by avoiding an enemy robot and staying in a certain area
 *
 * This skill is endless. It has two AntiTargetFeatures, one for the enemy
 * robot, a filled circle, and one for the area, a circle. Then theres a
 * PointShape Targetfeature for the area center. The enemy robot, the area
 * center and the radius are required parameters. There a 4 config parameters
 * for the influence distances, the radius of the enemy robot and the weight of
 * the area center. If the robot leaves the area, the circles weight is set to
 * 0.
 *
 * @p related_robot enemy_robot<br>
 * @p required_point area_center<br>
 * @p required_double radius<br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class RunFreeBuild : public SkillBuilder
{
   public:
    RunFreeBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills