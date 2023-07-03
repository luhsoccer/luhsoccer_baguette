#pragma once

#include "local_planner/robot_dynamic.hpp"
#include "local_planner/skills/task.hpp"
#include "local_planner/skills/skill.hpp"
#include "config/config_store.hpp"
#include "local_planner/agent_history.hpp"
namespace luhsoccer::local_planner {
class AvoidanceManager;
constexpr double DEFAULT_CONTROL_FREQ = 100;

class Agent {
   public:
    enum class AgentState { IDLE, RUNNING, FINISHED, ABORTED, ERROR };
    enum class PredictionExitStatus {
        MAX_PREDICTION_STEPS_REACHED,
        PRECALCULATION_FINISHED,
        ABORTED,
        GOAL_REACHED,
        COLLIDED
    };

    Agent(RobotDynamic robot_dynamic, const RobotIdentifier& robot, int agent_id,
          const std::shared_ptr<AvoidanceManager>& avoidance_manager, double control_freq = DEFAULT_CONTROL_FREQ,
          bool real_agent = false);

    bool setTask(const Skill* skill, const TaskData& task_data);

    std::optional<TaskData> getTaskData() { return this->task_data; }

    void updateTask(const TaskData& td) { this->task_data = td; }

    std::pair<std::optional<robot_interface::RobotCommand>, AgentState> calcCurrentCommandMessage(
        const std::shared_ptr<const transform::WorldModel>& wm, const time::TimePoint& time = time::TimePoint(0));

    [[nodiscard]] AgentState getCurrentState() const { return this->state; };

    bool cancelCurrentTask();

    std::vector<Marker> getMarkers(const std::shared_ptr<const transform::WorldModel>& wm,
                                   const time::TimePoint& time = time::TimePoint(0));

    // PredictionExitStatus predict(int no_of_steps);

    // bool killPrediction();

    // AgentHistory history;

   private:
    // double calculatePredictionScore();

    // void calculateStep();

    // current task
    std::optional<TaskData> task_data;
    const Skill* skill;
    size_t current_step;
    AgentState state;
    bool step_returned_error{false};

    // robot config
    // bool real_agent;
    // int agent_id;
    time::Rate rate;
    RobotDynamic robot_dynamic;
    const config_provider::ConfigStore& cs;
    RobotIdentifier robot;
    std::shared_ptr<AvoidanceManager> am;
};
}  // namespace luhsoccer::local_planner