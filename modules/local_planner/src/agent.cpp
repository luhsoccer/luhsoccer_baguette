
#include <utility>

#include "local_planner/agent.hpp"
#include "config_provider/config_store_main.hpp"
namespace luhsoccer::local_planner {

Agent::Agent(RobotDynamic robot_dynamic, const RobotIdentifier& robot, int /*agent_id*/,
             const std::shared_ptr<AvoidanceManager>& avoidance_manager, double control_freq, bool /*real_agent*/)
    : skill(nullptr),
      current_step(0),
      state(AgentState::IDLE),
      //   real_agent(real_agent),
      //   agent_id(agent_id),
      rate(control_freq, fmt::format("{}AgentControlLoop", robot)),
      robot_dynamic(std::move(robot_dynamic)),
      cs(config_provider::ConfigProvider::getConfigStore()),
      robot(robot),
      am(avoidance_manager) {}

bool Agent::setTask(const Skill* skill, const TaskData& task_data) {
    if (!skill) return false;
    // check if agent is in valid state
    if (this->state != AgentState::IDLE && this->state != AgentState::ABORTED && this->state != AgentState::FINISHED)
        return false;

    if (!skill->taskDataValid(task_data)) return false;

    this->skill = skill;
    this->task_data = task_data;
    this->state = AgentState::RUNNING;
    this->current_step = 0;
    return true;
}

bool Agent::cancelCurrentTask() {
    if (this->state == AgentState::RUNNING) {
        this->state = AgentState::ABORTED;
        return true;
    }
    return false;
}

std::pair<std::optional<robot_interface::RobotCommand>, Agent::AgentState> Agent::calcCurrentCommandMessage(
    const std::shared_ptr<const transform::WorldModel>& wm, const time::TimePoint& time) {
    if (this->state != AgentState::RUNNING || !this->skill || !this->task_data) return {std::nullopt, this->state};

    auto [step_state, msg] =
        this->skill->calcCommandMessage(wm, this->task_data.value(), this->robot, time, this->current_step, this->am);

    switch (step_state) {
        case Skill::SkillState::RUNNING:
            break;
        case Skill::SkillState::NEXT_STEP:
            this->current_step++;
            break;
        case Skill::SkillState::FINISHED:
            this->state = AgentState::FINISHED;
            break;
        case Skill::SkillState::ABORT:
            this->state = AgentState::ABORTED;
            break;
        case Skill::SkillState::ERROR:
            if (this->step_returned_error) this->state = AgentState::ERROR;
            break;
    }
    this->step_returned_error = step_state == Skill::SkillState::ERROR;

    return {msg, this->state};
}

std::vector<Marker> Agent::getMarkers(const std::shared_ptr<const transform::WorldModel>& wm,
                                      const time::TimePoint& time) {
    if (this->state != AgentState::RUNNING || !this->skill || !this->task_data) return {};

    return this->skill->getVisualizationMarkers(wm, this->task_data.value(), this->robot, time, this->current_step);
}

// void Agent::calculateStep() {
//     const AgentHistory::Step& latest_state = this->history.getLatestStep();
//     auto msg = this->calcCurrentCommandMessage(this->wm, latest_state.stamp);
//     RobotState state = convertToRobotState(latest_state.position, latest_state.velocity);
// }

}  // namespace luhsoccer::local_planner