#include "skill_books/bod_skill_book/halt.hpp"
// include components here
#include "local_planner_components/steps/dribbler_step.hpp"
#include "local_planner_components/steps/wait_step.hpp"

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