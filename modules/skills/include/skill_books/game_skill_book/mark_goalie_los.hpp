#pragma once

#include "skills/skill_builder.hpp"
#include "robot_control/components/component_util.hpp"

namespace luhsoccer::skills {

using namespace robot_control;
// NOLINTNEXTLINE(readability-identifier-naming)
using TD_Pos = ComponentPosition::TaskDataType;

/**
 * @brief Short description
 *
 * Detailed description
 *
 * @p related_robot <br>
 * @p required_point <br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class MarkGoalieLosBuild : public SkillBuilder {
   public:
    MarkGoalieLosBuild();

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};
}  // namespace luhsoccer::skills