
#include "skill_books/skill_tester.hpp"
#include "skill_books/skill_library.hpp"
#include "time/time.hpp"
#include "robot_control/robot_control_module.hpp"
#include "robot_control/skills/skill.hpp"
#include "robot_control/skills/task_data.hpp"
namespace luhsoccer::skills {

void SkillTester::loop(std::atomic_bool& should_run) {
    const time::Duration start_delay(10.0);
    time::TimePoint start = time::now();
    time::Rate rate(2);

    while (should_run && (time::now() - start) < start_delay) {
        rate.sleep();
    }
    auto visible_robots = this->wm->getVisibleRobots<Team::ALLY>();
    if (visible_robots.size() < 2) return;

    {
        RobotIdentifier robot = visible_robots[2];
        robot_control::TaskData td(robot);
        constexpr double TARGET_X = 0.3;
        td.required_positions.emplace_back(this->wm->getGlobalFrame(), TARGET_X, 0, 0);
        const robot_control::Skill& skill = this->skill_lib.getSkill(GameSkillNames::GO_TO_POINT);
        if (skill.taskDataValid(td)) {
            if (this->robot_control_module.setTask(&skill, td)) {
                this->logger.info("Successfully set skill '{}' for {}", skill.name, robot);
            } else {
                this->logger.warning("Failed to set skill '{}' for {}", skill.name, robot);
            }
        }
    }
}
}  // namespace luhsoccer::skills