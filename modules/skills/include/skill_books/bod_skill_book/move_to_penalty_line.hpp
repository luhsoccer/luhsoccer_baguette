#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Move to a penalty line, which one depends on required_bool
 *
 * If the required_bool is true, the robot moves to the penalty line on the ally
 * side, otherwise to the penalty line on the enemy side.
 * This is the calculation for penalty line distance:
 * 6.0DivB + 1.0m behind ball + 0.5m puffer
 *
 * @p related_robot <br>
 * @p required_point <br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool PenaltyShootOnAllyGoal<br>
 * @p required_string <br>
 */
class MoveToPenaltyLineBuild : public SkillBuilder
{
   public:
    MoveToPenaltyLineBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills