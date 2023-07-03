#include "scenario/scenario.hpp"
#include "simulation_interface/simulation_interface.hpp"
#include "local_planner/local_planner_module.hpp"
namespace luhsoccer::scenario {

void Scenario::setRobotToPosition(const std::shared_ptr<const transform::WorldModel>& wm,
                                  simulation_interface::SimulationInterface& simulation_interface,
                                  local_planner::LocalPlannerModule& local_planner_module,
                                  const skills::BodSkillBook& skill_book, const RobotIdentifier& id,
                                  Eigen::Affine2d position, bool present, bool real_live_mode) {
    if (real_live_mode) {
        if (this->ignore_position_in_real_live) return;
        if (!present) {
            // calc pose out of field
            transform::FieldData field_data = wm->getFieldData();
            position = Eigen::Translation2d(field_data.max_robot_radius * 3 * static_cast<double>(id.id),
                                            -field_data.size.y() / 2 + field_data.max_robot_radius * 2);
        }
        local_planner::TaskData td(id);
        td.required_positions = {{"", position}};
        local_planner_module.setTask(&skill_book.getSkill(skills::BodSkillNames::GO_TO_POINT), td);
    } else {
        simulation_interface.teleportRobot(id.id, id.team == Team::ALLY ? TeamColor::BLUE : TeamColor::YELLOW, position,
                                           {0.0, 0.0, 0.0}, present);
    }
}

bool Scenario::setup(const std::shared_ptr<const transform::WorldModel>& wm,
                     simulation_interface::SimulationInterface& simulation_interface,
                     local_planner::LocalPlannerModule& local_planner_module, const skills::BodSkillBook& skill_book,
                     bool real_live_mode) {
    if (this->state != ScenarioState::CREATED && this->state != ScenarioState::FINISHED) return false;
    this->involved_ally_robots.clear();
    this->involved_enemy_robots.clear();

    auto do_for_both_colors = [this, &wm, &simulation_interface, &local_planner_module, &skill_book, &real_live_mode](
                                  const std::vector<transform::Position>& setup,
                                  const std::vector<RobotIdentifier>& available_robots,
                                  std::vector<RobotIdentifier>& involved_robots_vector) {
        auto ally_setup_entry = setup.begin();

        for (const auto& id : available_robots) {
            if (ally_setup_entry != setup.end()) {
                auto position = ally_setup_entry->getCurrentPosition(wm);
                if (position.has_value()) {
                    local_planner_module.cancelTask(id);
                    this->setRobotToPosition(wm, simulation_interface, local_planner_module, skill_book, id,
                                             position.value(), true, real_live_mode);
                    involved_robots_vector.push_back(id);
                }
                ally_setup_entry++;
            } else {
                this->setRobotToPosition(wm, simulation_interface, local_planner_module, skill_book, id,
                                         Eigen::Affine2d(), false, real_live_mode);
            }
        }
        if (ally_setup_entry != setup.end()) {
            LOG_ERROR(logger::Logger("ScenarioExecutor"), "Not enough robots on the field to execute scenario!");
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

    if (!do_for_both_colors(this->enemy_setup, enemy_ids, this->involved_enemy_robots)) return false;

    if (real_live_mode) {
        // wait on robots to arrive
        time::Rate rate(10, "ScenarioExecutor");
        while (true) {
            rate.sleep();
            bool robot_running = false;
            for (const auto& robot : this->involved_ally_robots) {
                auto state = local_planner_module.getState(robot);
                if (state.has_value() && state.value() == local_planner::LocalPlanner::LocalPlannerState::RUNNING) {
                    robot_running = true;
                    continue;
                }
            }
            for (const auto& robot : this->involved_enemy_robots) {
                auto state = local_planner_module.getState(robot);
                if (state.has_value() && state.value() == local_planner::LocalPlanner::LocalPlannerState::RUNNING) {
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

bool Scenario::execute(local_planner::LocalPlannerModule& local_planner_module,
                       const skills::BodSkillBook& skill_book) {
    if (this->state != ScenarioState::SETUP) return false;
    for (const auto& robot : this->involved_ally_robots) {
        local_planner_module.cancelTask(robot);
    }
    if (this->involved_ally_robots.size() < this->tasks.size()) return false;

    auto robot_it = this->involved_ally_robots.begin();

    for (const auto& task : this->tasks) {
        local_planner::TaskData td(*robot_it);
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
        const local_planner::Skill& skill = skill_book.getSkill(task.first);
        if (!local_planner_module.setTask(&skill, td)) return false;
    }
    this->state = ScenarioState::RUNNING;
    return true;
}

void Scenario::stop(local_planner::LocalPlannerModule& local_planner_module) {
    if (this->state != ScenarioState::RUNNING) return;
    for (const auto& robot : this->involved_ally_robots) {
        local_planner_module.cancelTask(robot);
    }
    this->state = ScenarioState::FINISHED;
}

bool Scenario::isFinished(local_planner::LocalPlannerModule& local_planner_module) {
    if (this->state != ScenarioState::RUNNING) return true;

    for (const auto& [robot, ending] : this->running_ally_robots) {
        if (ending) {
            auto state = local_planner_module.getState(robot);
            if (state.has_value() && state == local_planner::LocalPlanner::LocalPlannerState::RUNNING) return false;
        }
    }

    // cancel all not ending skills
    for (const auto& [robot, ending] : this->running_ally_robots) {
        if (!ending) {
            local_planner_module.cancelTask(robot);
        }
    }

    // clean up
    this->involved_ally_robots.clear();
    this->running_ally_robots.clear();
    this->state = ScenarioState::FINISHED;

    return true;
}
}  // namespace luhsoccer::scenario