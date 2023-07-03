#pragma once

#include "skills/skill_builder.hpp"
#include "local_planner_components/steps/drive_step.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/shapes/point_shape.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"

namespace luhsoccer::skills
{

SKILL_DEFS

/**
 * @brief Get the ball in the dribbler
 *
 * Sets the dribbler mode to high, the drives close to the ball, the radius is
 * defined by a config, the drive closer to the ball to get it in the dribbler.
 *
 * @p related_robot <br>
 * @p required_point <br>
 * @p required_double <br>
 * @p required_int <br>
 * @p required_bool <br>
 * @p required_string <br>
 */
class GetBallBuild : public SkillBuilder
{
   public:
    GetBallBuild() : SkillBuilder("GetBall"){};

   private:
    void buildImpl(const config_provider::ConfigStore& cs) override;
};

}  // namespace luhsoccer::skills