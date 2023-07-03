#pragma once

#include "local_planner/skills/abstract_component.hpp"
#include "local_planner/skills/abstract_feature.hpp"
#include "local_planner/simulator.hpp"

namespace luhsoccer {
namespace marker {
class MarkerService;
}
namespace local_planner {
class ThreadPool;
class Skill;
class AvoidanceManager;

class SimulationManager : public AbstractComponent {
   public:
    SimulationManager(const SimulationManager&) = delete;
    SimulationManager(SimulationManager&&) = delete;
    SimulationManager& operator=(const SimulationManager&) = delete;
    SimulationManager& operator=(SimulationManager&&) = delete;
    SimulationManager(ThreadPool& thread_pool, marker::MarkerService& ms,
                      const std::shared_ptr<const transform::WorldModel>& real_wm,
                      const std::shared_ptr<AvoidanceManager>& am);
    ~SimulationManager() override;

    void setNewTask(const RobotIdentifier& robot, const Skill* skill, const TaskData& td);
    // std::vector<bool> multiAgentCallback(const std::vector<std::shared_ptr<const AbstractCFObstacle>>& features,
    //                                      const Eigen::Vector2d& goal_vec,
    //                                      const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
    //                                      const RobotIdentifier& robot, time::TimePoint time);

    void startSkillSimulation(const RobotIdentifier& robot, const Skill& skill, const TaskData& td,
                              const ResultCallback& result_callback, const time::TimePoint& start_time = time::now(),
                              std::shared_ptr<const transform::WorldModel> wm = nullptr);

    void startTaskDataSimulation(const RobotIdentifier& robot, const TaskData& td,
                                 const ResultCallback& result_callback, std::shared_ptr<const transform::WorldModel> wm,
                                 const std::optional<time::TimePoint>& start_from);

    SimulationResult runSyncSimulation(const RobotIdentifier& robot, const Skill& skill, const TaskData& td,
                                       const time::TimePoint& start_time = time::now(),
                                       std::shared_ptr<const transform::WorldModel> wm = nullptr);

    void stopSimulations() { this->should_run = false; }

   private:
    struct RotationVectorData {
        std::vector<bool> rotation_vectors;
        int score;
    };

    void startNewSimulation(const RobotIdentifier& robot, const std::shared_ptr<const transform::WorldModel>& wm,
                            const TaskData& td, const time::TimePoint& time);

    // void handleResult(unsigned long id, const SimulationResult& result, const time::TimePoint& start_time,
    //                   const RobotIdentifier& robot, LocalPlanner& local_planner);

    // static int calculateScore(const SimulationResult& result);

    std::unordered_map<RobotIdentifier, std::vector<std::shared_ptr<Simulator>>> simulators;
    std::unordered_map<RobotIdentifier, std::tuple<const Skill*, TaskData, time::TimePoint>> current_skills;
    std::vector<std::shared_ptr<Simulator>> independent_simulators;

    std::shared_mutex result_mtx;
    std::mutex simulator_mtx;
    std::atomic_bool should_run{true};
    ThreadPool& thread_pool;
    marker::MarkerService& ms;
    std::shared_ptr<const transform::WorldModel> real_wm;
    std::shared_ptr<AvoidanceManager> am;
    logger::Logger logger{"SimulationManager"};
};

// extern SimulationManager* simulation_manager;
}  // namespace local_planner
}  // namespace luhsoccer