#pragma once

#include "local_planner/agent.hpp"
#include "marker_service/marker_service.hpp"
#include "time/calc_time_stopwatch.hpp"
#include "local_planner/simulator.hpp"

namespace luhsoccer::local_planner {
class AvoidanceManager;
class LocalPlanner {
   public:
    enum class LocalPlannerState { IDLE, RUNNING, OUT_OF_FIELD, ERROR, CREATED, OFFLINE };
    LocalPlanner(std::shared_ptr<const transform::WorldModel> wm,
                 const std::shared_ptr<AvoidanceManager>& avoidance_manager,
                 robot_interface::RobotInterface& robot_interface, marker::MarkerService& ms,
                 const RobotIdentifier& robot, std::optional<double> controller_freq = std::nullopt);

    bool startLoop(const std::atomic_bool& should_run);

    void join();

    bool setTask(const Skill* skill, const TaskData& td);

    void setState(bool online);

    bool cancelCurrentTask();

    [[nodiscard]] LocalPlannerState getState() const { return this->state; }

    [[nodiscard]] const Skill* getCurrentSkill() {
        const std::lock_guard<std::mutex> lock(this->real_agent_mtx);
        return this->skill;
    }

    [[nodiscard]] std::optional<TaskData> getCurrentTaskData() {
        const std::lock_guard<std::mutex> lock(this->real_agent_mtx);
        return this->task_data;
    }

    bool updateTaskData(const TaskData& td) {
        if (this->state != LocalPlannerState::RUNNING) return false;
        const std::lock_guard<std::mutex> lock(this->real_agent_mtx);
        this->task_data = td;
        this->real_agent.updateTask(this->task_data.value());
        return true;
    }

   private:
    void loop(const std::atomic_bool& should_run);

    void checkIfRobotOnField();

    void sendCommand(const robot_interface::RobotCommand& msg);

    [[nodiscard]] robot_interface::RobotCommand getNullCmd() const;

    void sendNullCommand() { this->sendCommand(this->getNullCmd()); };

    std::optional<TaskData> task_data;
    const Skill* skill;
    std::shared_ptr<Simulator> simulator;
    std::atomic<LocalPlannerState> state;

    std::thread thread;

    Agent real_agent;
    std::mutex real_agent_mtx;

    std::shared_ptr<const transform::WorldModel> wm;
    robot_interface::RobotInterface& robot_interface;
    marker::MarkerService& ms;
    std::string marker_ns;
    RobotIdentifier robot;
    time::Rate rate;

    time::CalcTimeStopwatch sw;

    logger::Logger logger;
};
}  // namespace luhsoccer::local_planner

// Here is place for your own code