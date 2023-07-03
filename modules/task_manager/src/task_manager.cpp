#include "task_manager/task_manager.hpp"
#include "role_manager/role_manager.hpp"
#include "game_data_provider/game_data_provider.hpp"
#include "observer/continuous_observer.hpp"
#include "local_planner/local_planner.hpp"
#include "local_planner/local_planner_module.hpp"
#include "skill_books/bod_skill_book.hpp"

namespace luhsoccer::task_manager {

TaskManager::TaskManager(role_manager::RoleManager& manager, const game_data_provider::GameDataProvider& gdp,
                         const skills::BodSkillBook& book, local_planner::LocalPlannerModule& local_planner)
    : role_manager(manager), game_data_provider(gdp), book(book), local_planner(local_planner) {}

void TaskManager::loop(std::atomic_bool& /*should_run*/) {
    for (const auto& role : role_manager.getRoles()) {
        std::unique_lock lock(this->map_mutex);
        auto iter = this->task_callbacks.find(role);

        if (iter == this->task_callbacks.end()) {
            continue;
        }

        const auto robot_ids = role_manager.getRobotsForRole(role);

        if (robot_ids.empty()) {
            // LOG_INFO(logger, "Skipping assignment for role: {}. No robots given", role);
            continue;
        }

        const auto& callback = this->task_callbacks.at(role);

        if (!callback) {
            LOG_WARNING(logger, "Invalid function");
            continue;
        }

        std::vector<transform::RobotHandle> handles;
        handles.reserve(robot_ids.size());
        for (const auto& id : robot_ids) {
            handles.emplace_back(id, game_data_provider.getWorldModel());
        }

        const auto wm = game_data_provider.getWorldModel();
        const auto result = callback(handles, game_data_provider.getObserver(), game_data_provider.getWorldModel());
        for (const auto& kv : result) {
            updateSkill(kv.first, kv.second.first, kv.second.second);
        }
    }

    rate.sleep();
}

void TaskManager::updateSkill(RobotIdentifier robot, skills::BodSkillNames skill_name, local_planner::TaskData data) {
    const auto& skill = book.getSkill(skill_name);

    if (!skill.taskDataValid(data)) {
        LOG_WARNING(logger, "Won't set skill '{}'. TaskData invalid!", skill.name);
        return;
    }

    const auto state = local_planner.getState(robot);

    if (!state) {
        LOG_WARNING(logger, "Got no state from local planner. This should not happen");
        return;
    }

    if (state.value() == local_planner::LocalPlanner::LocalPlannerState::IDLE) {
        if (!local_planner.setTask(&skill, data)) {
            LOG_WARNING(logger, "Could not set skill for robot: {}", robot);
        } else {
            this->last_skill.insert_or_assign(robot, skill_name);
            this->last_task_data.insert_or_assign(robot, data);
        }
        return;
    } else {  // Compare last task data with current to find out if different
        // First check if skill names differ
        auto last_skill_iter = this->last_skill.find(robot);
        if (last_skill_iter == this->last_skill.end() || last_skill_iter->second != skill_name) {
            // Skill names differ, set new skill
            LOG_INFO(logger, "Got new skill for robot {}. Update task", robot);
            local_planner.cancelTask(robot);
            if (!local_planner.setTask(&skill, data)) {
                LOG_WARNING(logger, "Could not set skill for robot: {}", robot);
            } else {
                this->last_skill.insert_or_assign(robot, skill_name);
                this->last_task_data.insert_or_assign(robot, data);
            }
            return;
        }
        // If skill names are the same we must check if the task data differs
        auto last_td_iter = this->last_task_data.find(robot);
        if (last_td_iter == this->last_task_data.end() || !this->isTaskDataSame(last_td_iter->second, data)) {
            // Task data differ, set new skill
            LOG_INFO(logger, "Got new task data for robot {}. Update task", robot);
            local_planner.cancelTask(robot);
            if (!local_planner.setTask(&skill, data)) {
                LOG_WARNING(logger, "Could not set skill for robot: {}", robot);
            } else {
                this->last_skill.insert_or_assign(robot, skill_name);
                this->last_task_data.insert_or_assign(robot, data);
            }
            return;
        }
    }
}

void TaskManager::registerCallback(const std::string& role, const TaskCallback& callback) {
    std::lock_guard lock(this->map_mutex);

    if (!callback) {
        LOG_WARNING(logger, "Got function which is definitely not");
    }

    if (this->task_callbacks.contains(role)) {
        LOG_WARNING(logger, "Overwriting already existing task callback for role '{}'", role);
        this->task_callbacks.erase(role);
    }
    LOG_INFO(logger, "Registered callback for {}", role);

    this->task_callbacks.insert({role, callback});

    // this->task_callbacks.emplace(role, callback);
}

bool TaskManager::isTaskDataSame(const local_planner::TaskData& td1, const local_planner::TaskData& td2) const {
    if (td1.robot != td2.robot) {
        return false;
    }
    if (td1.related_robots != td2.related_robots) {
        return false;
    }
    if (td1.required_bools != td2.required_bools) {
        return false;
    }
    if (td1.required_ints != td2.required_ints) {
        return false;
    }
    if (td1.required_strings != td2.required_strings) {
        return false;
    }

    constexpr double POSITION_EPSILON = 0.05;
    constexpr double ROTATION_EPSILON = 5.0 / 180.0 * L_PI;
    constexpr double DOUBLE_EPSILON = 0.0001;

    if (td1.required_doubles.size() != td2.required_doubles.size()) {
        return false;
    }

    if (td1.required_positions.size() != td2.required_positions.size()) {
        return false;
    }

    for (size_t i = 0; i < td1.required_doubles.size(); ++i) {
        auto first = td1.required_doubles[i];
        auto second = td2.required_doubles[i];
        auto upper_bound = first + DOUBLE_EPSILON;
        auto lower_bound = first - DOUBLE_EPSILON;
        if (second < lower_bound || second > upper_bound) {
            return false;
        }
    }

    for (size_t i = 0; i < td1.required_positions.size(); ++i) {
        auto first = td1.required_positions[i];
        auto second = td2.required_positions[i];
        if (first.getFrame() != second.getFrame()) {
            return false;
        }
        if ((first.getPositionOffset().translation() - second.getPositionOffset().translation()).norm() >
            POSITION_EPSILON) {
            return false;
        }

        double a1 = Eigen::Rotation2Dd(first.getPositionOffset().rotation()).angle();
        double a2 = Eigen::Rotation2Dd(second.getPositionOffset().rotation()).angle();

        while (a1 < -L_PI) {
            a1 += 2 * L_PI;
        }

        while (a1 > L_PI) {
            a1 -= 2 * L_PI;
        }

        while (a2 < -L_PI) {
            a2 += 2 * L_PI;
        }

        while (a2 > L_PI) {
            a2 -= 2 * L_PI;
        }

        double diff = a1 - a2;

        if (diff > L_PI) {
            diff -= 2 * L_PI;
        }

        if (diff < -L_PI) {
            diff += 2 * L_PI;
        }

        diff = std::abs(diff);

        if (diff > ROTATION_EPSILON) {
            return false;
        }

        // todo
        /*       double heading_err = std::abs(Eigen::Rotation2Dd(first.getPositionOffset().rotation()).angle() -
                                              Eigen::Rotation2Dd(second.getPositionOffset().rotation()).angle());


                if (heading_err > L_PI) {
                    heading_err -= 2 * L_PI;
                }
                if (heading_err > ROTATION_EPSILON) {
                    return false;
                }*/
    }

    return true;
}

void TaskManager::stop() {
    std::lock_guard lock(this->map_mutex);
    this->task_callbacks.clear();
}

}  // namespace luhsoccer::task_manager