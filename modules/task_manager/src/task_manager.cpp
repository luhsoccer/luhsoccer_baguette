#include "task_manager/task_manager.hpp"
#include "game_data_provider/game_data_provider.hpp"
#include "skill_books/skill_library.hpp"
#include "robot_control/robot_control_module.hpp"
#include "config_provider/config_store_main.hpp"
#include "config/game_config.hpp"

namespace luhsoccer::task_manager {

TaskManager::TaskManager(const game_data_provider::GameDataProvider& gdp, const skills::SkillLibrary& skill_lib,
                         robot_control::RobotControlModule& robot_control)
    : game_data_provider(gdp), skill_library(skill_lib), robot_control(robot_control) {}

bool TaskManager::updateTask(SkillNames skill_name, robot_control::TaskData data, bool force) {
    if (!config_provider::ConfigProvider::getConfigStore().game_config.strategy_active) return false;

    const auto& skill = skill_library.getSkill(skill_name);
    const auto robot = data.robot;

    if (!skill.taskDataValid(data)) {
        logger.warning("Won't set skill '{}'. TaskData invalid!", skill.name);
        return false;
    }

    const auto state = robot_control.getState(robot);

    if (!state) {
        logger.warning("Got no state from local planner. This should not happen");
        return false;
    }

    if (state.value() == robot_control::RobotControllerState::IDLE) {
        if (!robot_control.setTask(&skill, data)) {
            logger.warning("Could not set skill for robot: {}", robot);
            return false;
        } else {
            this->last_skill.insert_or_assign(robot, skill_name);
            this->last_task_data.insert_or_assign(robot, data);
        }
    } else {  // Compare last task data with current to find out if different
        // First check if skill names differ
        auto last_skill_iter = this->last_skill.find(robot);
        if (last_skill_iter == this->last_skill.end() || last_skill_iter->second != skill_name) {
            // Skill names differ, set new skill
            logger.info("Got new skill for robot {}. Update task", robot);
            robot_control.cancelTask(robot);
            if (!robot_control.setTask(&skill, data)) {
                logger.warning("Could not set skill for robot: {}", robot);
                return false;
            } else {
                this->last_skill.insert_or_assign(robot, skill_name);
                this->last_task_data.insert_or_assign(robot, data);
            }
        }
        // If skill names are the same we must check if the task data differs
        auto last_td_iter = this->last_task_data.find(robot);
        if (last_td_iter == this->last_task_data.end() || !this->isTaskDataSame(last_td_iter->second, data) || force) {
            // Task data differ, set new skill
            logger.info("Got new task data for robot {}. Update task. Using force: {}", robot, force);
            robot_control.cancelTask(robot);
            if (!robot_control.setTask(&skill, data)) {
                logger.warning("Could not set skill for robot: {}", robot);
                return false;
            } else {
                this->last_skill.insert_or_assign(robot, skill_name);
                this->last_task_data.insert_or_assign(robot, data);
            }
        } else {
            return false;
        }
    }

    return true;
}

bool TaskManager::isTaskDataSame(const robot_control::TaskData& td1, const robot_control::TaskData& td2) const {
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

std::optional<SkillNames> TaskManager::getLastSkill(const RobotIdentifier& robot) const {
    auto iter = this->last_skill.find(robot);
    if (iter == this->last_skill.end()) {
        return std::nullopt;
    } else {
        return iter->second;
    }
}

std::optional<robot_control::TaskData> TaskManager::getLastTaskData(const RobotIdentifier& robot) const {
    auto iter = this->last_task_data.find(robot);
    if (iter == this->last_task_data.end()) {
        return std::nullopt;
    } else {
        return iter->second;
    }
}

}  // namespace luhsoccer::task_manager