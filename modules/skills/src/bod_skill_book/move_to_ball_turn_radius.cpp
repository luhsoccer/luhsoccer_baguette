#include "skill_books/bod_skill_book/move_to_ball_turn_radius.hpp"
// include components here

#include "bod_skill_book/move_to_ball_turn_radius_macro.hpp"
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/features/target_feature.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
#include "local_planner_components/steps/drive_step.hpp"

namespace luhsoccer::skills {

MoveToBallTurnRadiusBuild::MoveToBallTurnRadiusBuild()
    : SkillBuilder("MoveToBallTurnRadius",  //
                   {},                      //
                   {},                      //
                   {},                      //
                   {},                      //
                   {},                      //
                   {}){};

void MoveToBallTurnRadiusBuild::buildImpl(const config_provider::ConfigStore& cs) {
    MOVE_TO_BALL_TURN_RADIUS

    // Add skill definition here. Use addStep to add a step

    // end of skill
}
}  // namespace luhsoccer::skills