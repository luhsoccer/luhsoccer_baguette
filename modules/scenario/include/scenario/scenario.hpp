#pragma once

#include <string>
#include <utility>

#include "skill_books/skill_library.hpp"

#include "transform/position.hpp"

namespace luhsoccer {

namespace robot_control {
class RobotControlModule;
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

class ScenarioRobotSetup {
   public:
    ScenarioRobotSetup() : type(Type::NOT_INVOLVED), start_position(""){};
    ScenarioRobotSetup(transform::Position position) : type(Type::STATIC), start_position(std::move(position)){};
    ScenarioRobotSetup(const std::vector<std::pair<time::Duration, transform::Position>>& positions)
        : type(Type::MOVING), start_position(positions.front().second), positions(positions) {
        this->positions.erase(this->positions.begin());
    };

    void teleportRobot(const std::shared_ptr<const transform::WorldModel>& wm,
                       simulation_interface::SimulationInterface& simulation_interface,
                       robot_control::RobotControlModule& robot_control_module, const skills::SkillLibrary& skill_lib,
                       const RobotIdentifier& id, bool real_live_mode, bool ignore_position_in_real_live,
                       time::Duration time_since_start) const;

    [[nodiscard]] bool robotInvolved() const { return this->type != Type::NOT_INVOLVED; };

    [[nodiscard]] std::optional<time::Duration> getLatestTime() const {
        if (this->type == Type::MOVING) {
            return positions.back().first;
        }
        return std::nullopt;
    }
    [[nodiscard]] transform::Position getStartPosition() const { return start_position; }

   private:
    enum class Type { STATIC, MOVING, NOT_INVOLVED };
    Type type;
    transform::Position start_position;
    std::vector<std::pair<time::Duration, transform::Position>> positions;
};

class Scenario {
   public:
    Scenario(std::string name, std::vector<transform::Position> ally_setup, std::vector<ScenarioRobotSetup> enemy_setup,
             std::vector<std::pair<skills::SkillNames, ScenarioTaskData>> tasks,
             bool ignore_position_in_real_live = false)
        : name(std::move(name)),
          ally_setup(std::move(ally_setup)),
          enemy_setup(std::move(enemy_setup)),
          tasks(std::move(tasks)),
          ignore_position_in_real_live(ignore_position_in_real_live){};

    [[nodiscard]] std::string getName() const { return this->name; }
    bool setup(const std::shared_ptr<const transform::WorldModel>& wm,
               simulation_interface::SimulationInterface& simulation_interface,
               robot_control::RobotControlModule& robot_control_module, const skills::SkillLibrary& skill_lib,
               bool real_live_mode);
    bool execute(robot_control::RobotControlModule& robot_control_module, const skills::SkillLibrary& skill_lib);

    void teleportRobots(const std::shared_ptr<const transform::WorldModel>& wm,
                        simulation_interface::SimulationInterface& simulation_interface,
                        robot_control::RobotControlModule& robot_control_module, const skills::SkillLibrary& skill_lib,
                        bool real_live_mode, time::Duration time_since_start);

    void stop(robot_control::RobotControlModule& robot_control_module);

    bool isFinished(robot_control::RobotControlModule& robot_control_module, time::Duration time_since_start);

    [[nodiscard]] std::vector<RobotIdentifier> getInvolvedAllyRobots() const { return this->involved_ally_robots; }
    [[nodiscard]] std::vector<RobotIdentifier> getInvolvedEnemyRobots() const { return this->involved_enemy_robots; }

    [[nodiscard]] std::vector<std::pair<RobotIdentifier, bool>> getRunningRobots() const {
        return this->running_ally_robots;
    }

    void static setRobotToPosition(const std::shared_ptr<const transform::WorldModel>& wm,
                                   simulation_interface::SimulationInterface& simulation_interface,
                                   robot_control::RobotControlModule& robot_control_module,
                                   const skills::SkillLibrary& skill_lib, const RobotIdentifier& id,
                                   Eigen::Affine2d position, bool present, bool real_live_mode,
                                   bool ignore_position_in_real_live);

   private:
    // void setRobotToPosition(const std::shared_ptr<const transform::WorldModel>& wm,
    //                         simulation_interface::SimulationInterface& simulation_interface,
    //                         local_planner::LocalPlannerModule& robot_control_module,
    //                         const skills::BodSkillBook& skill_book, const RobotIdentifier& id, Eigen::Affine2d
    //                         position, bool present, bool real_live_mode);
    std::string name;

    std::vector<transform::Position> ally_setup;
    std::vector<ScenarioRobotSetup> enemy_setup;
    std::vector<std::pair<skills::SkillNames, ScenarioTaskData>> tasks;

    enum class ScenarioState { CREATED, SETUP, RUNNING, FINISHED };
    ScenarioState state{ScenarioState::CREATED};
    std::vector<RobotIdentifier> involved_ally_robots{};
    std::vector<RobotIdentifier> involved_enemy_robots{};
    std::vector<std::pair<RobotIdentifier, bool>> running_ally_robots{};
    bool ignore_position_in_real_live;
};
}  // namespace scenario
}  // namespace luhsoccer