
#include "skill_books/skill_tester.hpp"
#include "time/time.hpp"
#include "local_planner/local_planner_module.hpp"
#include "local_planner/skills/skill.hpp"
#include "local_planner/skills/task.hpp"
#include "skill_books/bod_skill_book.hpp"
#include "transform/world_model.hpp"
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
        local_planner::TaskData td(robot);
        constexpr double TARGET_X = 0.3;
        td.required_positions.emplace_back(this->wm->getGlobalFrame(), TARGET_X, 0, 0);
        const local_planner::Skill& skill = this->skill_book.getSkill(BodSkillNames::GO_TO_POINT);
        if (skill.taskDataValid(td)) {
            if (this->local_planner_module.setTask(&skill, td)) {
                LOG_INFO(this->logger, "Successfully set skill '{}' for {}", skill.name, robot);
            } else {
                LOG_WARNING(this->logger, "Failed to set skill '{}' for {}", skill.name, robot);
            }
        }
    }
}
}  // namespace luhsoccer::skills