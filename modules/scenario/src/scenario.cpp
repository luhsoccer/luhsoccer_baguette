#include "scenario/scenario.hpp"
#include "robot_control/skills/task_data.hpp"
#include "simulation_interface/simulation_interface.hpp"
#include "robot_control/robot_control_module.hpp"
namespace luhsoccer::scenario {

void ScenarioRobotSetup::teleportRobot(const std::shared_ptr<const transform::WorldModel>& wm,
                                       simulation_interface::SimulationInterface& simulation_interface,
                                       robot_control::RobotControlModule& robot_control_module,
                                       const skills::SkillLibrary& skill_lib, const RobotIdentifier& id,
                                       bool real_live_mode, bool ignore_position_in_real_live,
                                       time::Duration time_since_start) const {
    if (id.isAlly()) robot_control_module.cancelTask(id);

    if (this->type == Type::NOT_INVOLVED) {
        Scenario::setRobotToPosition(wm, simulation_interface, robot_control_module, skill_lib, id, Eigen::Affine2d(),
                                     false, real_live_mode, ignore_position_in_real_live);
        return;
    }
    if (time_since_start == time::Duration(0)) {
        auto position = this->start_position.getCurrentPosition(wm);
        if (position.has_value()) {
            Scenario::setRobotToPosition(wm, simulation_interface, robot_control_module, skill_lib, id,
                                         position.value(), true, real_live_mode, ignore_position_in_real_live);
        }
        return;
    }

    if (this->type == Type::MOVING) {
        if (time_since_start.asSec() < 0.0) return;
        // check if after last known pos:
        if (this->positions.back().first < time_since_start) {
            auto position = this->positions.back().second.getCurrentPosition(wm);
            if (position.has_value()) {
                Scenario::setRobotToPosition(wm, simulation_interface, robot_control_module, skill_lib, id,
                                             position.value(), true, real_live_mode, ignore_position_in_real_live);
            }
        }
        // interpolate position
        for (auto position_it = this->positions.begin(); position_it != this->positions.end(); ++position_it) {
            if (position_it->first > time_since_start) {
                auto second_position = position_it->second.getCurrentPosition(wm);
                std::optional<Eigen::Affine2d> first_position;
                if (position_it == this->positions.begin()) {
                    first_position = this->start_position.getCurrentPosition(wm);
                } else {
                    first_position = (position_it - 1)->second.getCurrentPosition(wm);
                }
                if (first_position.has_value() && second_position.has_value()) {
                    time::Duration time2 = position_it->first;
                    time::Duration time1;
                    if (position_it == this->positions.begin()) {
                        time1 = 0.0;
                    } else {
                        time1 = (position_it - 1)->first;
                    }
                    double dt = 0.0;
                    if (time1 != time2) {
                        dt = (time_since_start - time1).count() / (double)(time2 - time1).count();
                    }
                    Eigen::Translation2d translation(first_position->translation() +
                                                     (second_position->translation() - first_position->translation()) *
                                                         dt);

                    Scenario::setRobotToPosition(wm, simulation_interface, robot_control_module, skill_lib, id,
                                                 translation * first_position->rotation(), true, real_live_mode,
                                                 ignore_position_in_real_live);
                }
                break;
            }
        }
        return;
    }
}

void Scenario::setRobotToPosition(const std::shared_ptr<const transform::WorldModel>& wm,
                                  simulation_interface::SimulationInterface& simulation_interface,
                                  robot_control::RobotControlModule& robot_control_module,
                                  const skills::SkillLibrary& skill_lib, const RobotIdentifier& id,
                                  Eigen::Affine2d position, bool present, bool real_live_mode,
                                  bool ignore_position_in_real_live) {
    if (real_live_mode) {
        if (ignore_position_in_real_live) return;
        if (!present) {
            // calc pose out of field
            transform::FieldData field_data = wm->getFieldData();
            position = Eigen::Translation2d(field_data.max_robot_radius * 3 * static_cast<double>(id.id),
                                            -field_data.size.y() / 2 + field_data.max_robot_radius * 2);
        }
        robot_control::TaskData td(id);
        td.required_positions = {{"", position}};
        robot_control_module.setTask(&skill_lib.getSkill(skills::GameSkillNames::GO_TO_POINT), td);
    } else {
        simulation_interface.teleportRobot(id.id, id.team == Team::ALLY ? TeamColor::BLUE : TeamColor::YELLOW, position,
                                           {0.0, 0.0, 0.0}, present);
    }
}

bool Scenario::setup(const std::shared_ptr<const transform::WorldModel>& wm,
                     simulation_interface::SimulationInterface& simulation_interface,
                     robot_control::RobotControlModule& robot_control_module, const skills::SkillLibrary& skill_lib,
                     bool real_live_mode) {
    if (this->state != ScenarioState::CREATED && this->state != ScenarioState::FINISHED) return false;
    this->involved_ally_robots.clear();
    this->involved_enemy_robots.clear();

    auto do_for_both_colors = [this, &wm, &simulation_interface, &robot_control_module, &skill_lib, &real_live_mode](
                                  const std::vector<transform::Position>& setup,
                                  const std::vector<RobotIdentifier>& available_robots,
                                  std::vector<RobotIdentifier>& involved_robots_vector) {
        auto ally_setup_entry = setup.begin();

        for (const auto& id : available_robots) {
            if (ally_setup_entry != setup.end()) {
                auto position = ally_setup_entry->getCurrentPosition(wm);
                if (position.has_value()) {
                    robot_control_module.cancelTask(id);
                    this->setRobotToPosition(wm, simulation_interface, robot_control_module, skill_lib, id,
                                             position.value(), true, real_live_mode,
                                             this->ignore_position_in_real_live);
                    involved_robots_vector.push_back(id);
                }
                ally_setup_entry++;
            } else {
                this->setRobotToPosition(wm, simulation_interface, robot_control_module, skill_lib, id,
                                         Eigen::Affine2d(), false, real_live_mode, this->ignore_position_in_real_live);
            }
        }
        if (ally_setup_entry != setup.end()) {
            logger::Logger("ScenarioExecutor").error("Not enough robots on the field to execute scenario!");
            return false;
        }

        return true;
    };

    std::vector<RobotIdentifier> ally_ids;
    if (real_live_mode) {
        ally_ids = wm->getVisibleRobots<Team::ALLY>();
    } else {
        ally_ids = wm->getPossibleRobots<Team::ALLY>();
    }

    if (!do_for_both_colors(this->ally_setup, ally_ids, this->involved_ally_robots)) return false;

    std::vector<RobotIdentifier> enemy_ids;
    if (real_live_mode) {
        enemy_ids = wm->getVisibleRobots<Team::ENEMY>();
    } else {
        enemy_ids = wm->getPossibleRobots<Team::ENEMY>();
    }
    std::vector<transform::Position> initial_enemy_setup;
    initial_enemy_setup.reserve(this->enemy_setup.size());
    for (const auto& setup : this->enemy_setup) {
        initial_enemy_setup.push_back(setup.getStartPosition());
    }

    if (!do_for_both_colors(initial_enemy_setup, enemy_ids, this->involved_enemy_robots)) return false;

    if (real_live_mode) {
        // wait on robots to arrive
        time::Rate rate(10, "ScenarioExecutor");
        while (true) {
            rate.sleep();
            bool robot_running = false;
            for (const auto& robot : this->involved_ally_robots) {
                auto state = robot_control_module.getState(robot);
                if (state.has_value() && state.value() == robot_control::RobotControllerState::RUNNING) {
                    robot_running = true;
                    continue;
                }
            }
            for (const auto& robot : this->involved_enemy_robots) {
                auto state = robot_control_module.getState(robot);
                if (state.has_value() && state.value() == robot_control::RobotControllerState::RUNNING) {
                    robot_running = true;
                    continue;
                }
            }
            if (!robot_running) break;
        }
    }

    this->state = ScenarioState::SETUP;
    return true;
}

bool Scenario::execute(robot_control::RobotControlModule& robot_control_module, const skills::SkillLibrary& skill_lib) {
    if (this->state != ScenarioState::SETUP) return false;
    for (const auto& robot : this->involved_ally_robots) {
        robot_control_module.cancelTask(robot);
    }
    if (this->involved_ally_robots.size() < this->tasks.size()) return false;

    auto robot_it = this->involved_ally_robots.begin();

    for (const auto& task : this->tasks) {
        robot_control::TaskData td(*robot_it);
        for (const auto& [id, team] : task.second.related_robots) {
            if (team == Team::ALLY) {
                td.related_robots.push_back(this->involved_ally_robots[id]);
            } else {
                td.related_robots.push_back(this->involved_enemy_robots[id]);
            }
        }
        td.required_positions = task.second.required_positions;
        td.required_bools = task.second.required_bools;
        td.required_doubles = task.second.required_doubles;
        td.required_ints = task.second.required_ints;
        td.required_strings = task.second.required_strings;
        this->running_ally_robots.emplace_back(*robot_it, task.second.ending);
        robot_it++;
        const robot_control::Skill& skill = skill_lib.getSkill(task.first);
        if (!robot_control_module.setTask(&skill, td)) return false;
    }
    this->state = ScenarioState::RUNNING;
    return true;
}

void Scenario::stop(robot_control::RobotControlModule& robot_control_module) {
    if (this->state != ScenarioState::RUNNING) return;
    for (const auto& robot : this->involved_ally_robots) {
        robot_control_module.cancelTask(robot);
    }
    this->state = ScenarioState::FINISHED;
}

void Scenario::teleportRobots(const std::shared_ptr<const transform::WorldModel>& wm,
                              simulation_interface::SimulationInterface& simulation_interface,
                              robot_control::RobotControlModule& robot_control_module,
                              const skills::SkillLibrary& skill_lib, bool real_live_mode,
                              time::Duration time_since_start) {
    auto involved_enemy_robots_it = this->involved_enemy_robots.begin();
    for (const auto& setup : this->enemy_setup) {
        setup.teleportRobot(wm, simulation_interface, robot_control_module, skill_lib, *involved_enemy_robots_it,
                            real_live_mode, this->ignore_position_in_real_live, time_since_start);
        involved_enemy_robots_it++;
        if (involved_enemy_robots_it == this->involved_enemy_robots.end()) {
            return;
        }
    }
}

bool Scenario::isFinished(robot_control::RobotControlModule& robot_control_module, time::Duration time_since_start) {
    if (this->state != ScenarioState::RUNNING) return true;

    for (const auto& [robot, ending] : this->running_ally_robots) {
        if (ending) {
            auto state = robot_control_module.getState(robot);
            if (state.has_value() && state == robot_control::RobotControllerState::RUNNING) return false;
        }
    }

    for (const auto& setup : this->enemy_setup) {
        auto time = setup.getLatestTime();
        if (time.has_value() && time > time_since_start) return false;
    }

    // cancel all not ending skills
    for (const auto& [robot, ending] : this->running_ally_robots) {
        if (!ending) {
            robot_control_module.cancelTask(robot);
        }
    }

    // clean up
    this->involved_ally_robots.clear();
    this->running_ally_robots.clear();
    this->state = ScenarioState::FINISHED;

    return true;
}
}  // namespace luhsoccer::scenario