#pragma once

#include <utility>

#include "robot_control/skills/task_data.hpp"

namespace luhsoccer::robot_control {

class Skill;

struct SimulationTask {
    SimulationTask(const Skill* skill, TaskData task_data) : skill(skill), task_data(std::move(task_data)){};
    SimulationTask(const SimulationTask& other) = default;
    SimulationTask& operator=(const SimulationTask& other) = default;
    SimulationTask(SimulationTask&& other) = default;
    SimulationTask& operator=(SimulationTask&& other) = default;
    ~SimulationTask() = default;
    const Skill* skill{};
    TaskData task_data;
};

struct SimulationResult {
    std::shared_ptr<const transform::WorldModel> wm;
    std::unordered_map<RobotIdentifier, SimulationTask> tasks;
    time::TimePoint start_time;
    time::TimePoint end_time;
};

using ResultCallback = std::function<void(unsigned long id, const SimulationResult& result)>;

}  // namespace luhsoccer::robot_control