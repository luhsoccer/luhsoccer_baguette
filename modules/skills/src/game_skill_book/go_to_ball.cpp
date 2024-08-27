#include "skill_books/game_skill_book/go_to_ball.hpp"
#include "go_to_ball_steps.hpp"
#include "robot_control/components/steps/dribbler_step.hpp"
// include components here

namespace luhsoccer::skills {

GoToBallBuild::GoToBallBuild()
    : SkillBuilder("GoToBall",  //
                   {},          //
                   {},          //
                   {},          //
                   {},          //
                   {},          //
                   {}){};

void GoToBallBuild::buildImpl(const config_provider::ConfigStore& cs) {
    // Add skill definition here. Use addStep to add a step
    addStep(DribblerStep(robot_interface::DribblerMode::LOW));
    addSteps(getGoToBallSteps(cs));

    // end of skill
}
}  // namespace luhsoccer::skills