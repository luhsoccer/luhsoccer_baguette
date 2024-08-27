#include "skill_books/game_skill_book/halt.hpp"
#include "robot_control/components/steps/dribbler_step.hpp"
#include "robot_control/components/steps/wait_step.hpp"
// include components here

namespace luhsoccer::skills {

HaltBuild::HaltBuild()
    : SkillBuilder("Halt",  //
                   {},      //
                   {},      //
                   {},      //
                   {},      //
                   {},      //
                   {}){};

void HaltBuild::buildImpl(const config_provider::ConfigStore& /*cs*/) {
    // Add skill definition here. Use addStep to add a step
    addStep(DribblerStep(robot_interface::DribblerMode::OFF));
    addStep(WaitStep(WAIT_DURATION, 1.0));
    addStep(DribblerStep(robot_interface::DribblerMode::OFF));
    addStep(WaitStep(WAIT_BOOL, true));

    // end of skill
}
}  // namespace luhsoccer::skills