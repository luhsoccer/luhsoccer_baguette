#pragma once

#include <string>
#include <utility>

#include "local_planner/skills/task.hpp"
#include "local_planner/skills/skill.hpp"

#include "skill_books/bod_skill_book.hpp"

namespace luhsoccer {

namespace local_planner {
class LocalPlannerModule;
}

namespace simulation_interface {
class SimulationInterface;
}
namespace scenario {

struct ScenarioTaskData {
    explicit ScenarioTaskData(bool ending = true, std::vector<std::pair<size_t, Team>> related_robots = {},
                              std::vector<transform::Position> required_positions = {},
                              std::vector<int> required_ints = {}, std::vector<double> required_doubles = {},
                              std::vector<bool> required_bools = {}, std::vector<std::string> required_strings = {})

        : ending(ending),
          related_robots(std::move(related_robots)),
          required_positions(std::move(required_positions)),
          required_ints(std::move(required_ints)),
          required_doubles(std::move(required_doubles)),
          required_bools(std::move(required_bools)),
          required_strings(std::move(required_strings)){};
    bool ending;
    std::vector<std::pair<size_t, Team>> related_robots;
    std::vector<transform::Position> required_positions;
    std::vector<int> required_ints;
    std::vector<double> required_doubles;
    std::vector<bool> required_bools;
    std::vector<std::string> required_strings;
};
class Scenario {
   public:
    Scenario(std::string name, std::vector<transform::Position> ally_setup,
             std::vector<transform::Position> enemy_setup,
             std::vector<std::pair<skills::BodSkillNames, ScenarioTaskData>> tasks,
             bool ignore_position_in_real_live = false)
        : name(std::move(name)),
          ally_setup(std::move(ally_setup)),
          enemy_setup(std::move(enemy_setup)),
          tasks(std::move(tasks)),
          ignore_position_in_real_live(ignore_position_in_real_live){};

    [[nodiscard]] std::string getName() const { return this->name; }
    bool setup(const std::shared_ptr<const transform::WorldModel>& wm,
               simulation_interface::SimulationInterface& simulation_interface,
               local_planner::LocalPlannerModule& local_planner_module, const skills::BodSkillBook& skill_book,
               bool real_live_mode);
    bool execute(local_planner::LocalPlannerModule& local_planner_module, const skills::BodSkillBook& skill_book);

    void stop(local_planner::LocalPlannerModule& local_planner_module);

    bool isFinished(local_planner::LocalPlannerModule& local_planner_module);

    [[nodiscard]] std::vector<RobotIdentifier> getInvolvedAllyRobots() const { return this->involved_ally_robots; }
    [[nodiscard]] std::vector<RobotIdentifier> getInvolvedEnemyRobots() const { return this->involved_enemy_robots; }

    [[nodiscard]] std::vector<std::pair<RobotIdentifier, bool>> getRunningRobots() const {
        return this->running_ally_robots;
    }

   private:
    void setRobotToPosition(const std::shared_ptr<const transform::WorldModel>& wm,
                            simulation_interface::SimulationInterface& simulation_interface,
                            local_planner::LocalPlannerModule& local_planner_module,
                            const skills::BodSkillBook& skill_book, const RobotIdentifier& id, Eigen::Affine2d position,
                            bool present, bool real_live_mode);
    std::string name;

    std::vector<transform::Position> ally_setup;
    std::vector<transform::Position> enemy_setup;
    std::vector<std::pair<skills::BodSkillNames, ScenarioTaskData>> tasks;

    enum class ScenarioState { CREATED, SETUP, RUNNING, FINISHED };
    ScenarioState state{ScenarioState::CREATED};
    std::vector<RobotIdentifier> involved_ally_robots{};
    std::vector<RobotIdentifier> involved_enemy_robots{};
    std::vector<std::pair<RobotIdentifier, bool>> running_ally_robots{};
    bool ignore_position_in_real_live;
};
}  // namespace scenario
}  // namespace luhsoccer