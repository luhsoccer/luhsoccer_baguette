#pragma once
#include <memory>
#include <utility>
#include "local_planner/skills/task.hpp"
#include "robot_identifier.hpp"
#include "local_planner/robot_dynamic.hpp"
#include "local_planner/agent.hpp"
#include "robot_interface/robot_interface.hpp"
#include "config_provider/config_store_main.hpp"
#include "skills/skill_util.hpp"

namespace luhsoccer {

namespace transform {
class WorldModel;
}

namespace local_planner {

class Skill;
class ThreadPool;
class AvoidanceManager;
struct SimulationTask {
    const Skill* skill;
    TaskData task_data;
    std::map<unsigned long, bool> rotation_vectors;
};

struct SimulationResult {
    std::shared_ptr<const transform::WorldModel> wm;
    std::unordered_map<RobotIdentifier, SimulationTask> tasks;
    time::TimePoint start_time;
    time::TimePoint end_time;
};
using ResultCallback = std::function<void(unsigned long id, const SimulationResult& result)>;
constexpr double DEFAULT_SIMULATION_FREQUENCY = 30.0;

class Simulator {
   public:
    Simulator(ThreadPool& thread_pool, const std::shared_ptr<const transform::WorldModel>& base_wm,
              const std::shared_ptr<AvoidanceManager>& am, RobotDynamic robot_dynamic = RobotDynamic(),
              double simulation_frequency = DEFAULT_SIMULATION_FREQUENCY);

    void setTasks(std::unordered_map<RobotIdentifier, SimulationTask> tasks) {
        if (this->state == SimulatorState::RUNNING) return;
        this->tasks = std::move(tasks);
    }

    static unsigned long startSimulation(const std::shared_ptr<Simulator>& simulator,
                                         const std::atomic_bool& should_run, const time::TimePoint& start_time,
                                         const std::optional<time::TimePoint>& start_from = std::nullopt);

    void stopSimulation() { this->should_run = false; }

    void setResultCallback(const ResultCallback& callback) {
        if (this->state == SimulatorState::RUNNING) return;
        this->result_callback = callback;
    };

    SimulationResult runSyncSimulation(const std::atomic_bool& should_run, const time::TimePoint& start_time,
                                       const std::optional<time::TimePoint>& start_from = std::nullopt);

   private:
    void runSimulation(const std::atomic_bool& should_run);

    std::unordered_map<RobotIdentifier, robot_interface::RobotCommand> calcCommands();

    void applyRobotCommands(const std::unordered_map<RobotIdentifier, robot_interface::RobotCommand>& commands,
                            const time::TimePoint& current_time);

    bool simulationValid(
        const DoubleLocalPlannerParam& /*max_valid_dist*/ = localPlannerConfig().simulation_max_deviation) {
        return true;
    };

    static bool moveFrame(const std::shared_ptr<transform::WorldModel>& wm, const std::string& frame,
                          const time::TimePoint& current_time, const time::Duration& step_duration,
                          const Eigen::Vector3d& acceleration = {0.0, 0.0, 0.0});

    static bool applyRobotCommandVelocity(const std::shared_ptr<transform::WorldModel>& wm,
                                          const RobotDynamic& robot_dynamic, RobotIdentifier robot,
                                          const time::TimePoint& current_time, const time::Duration& step_duration,
                                          const Eigen::Vector3d& local_command_velocity);

    static Eigen::Vector3d getRobotAccelerationFromCommand(const std::shared_ptr<const transform::WorldModel>& wm,
                                                           const RobotIdentifier& robot,
                                                           const robot_interface::RobotCommand& command,
                                                           const time::Duration& step_duration,
                                                           const time::TimePoint& time = time::TimePoint(0));
    static Eigen::Vector3d getRobotCommandVelocityFromCommand(const robot_interface::RobotCommand& command);

    enum class SimulatorState { CREATED, RUNNING, FINISHED };
    std::atomic<SimulatorState> state{SimulatorState::CREATED};
    // configuration
    std::shared_ptr<transform::WorldModel> wm;
    std::shared_ptr<const transform::WorldModel> base_wm;
    std::shared_ptr<AvoidanceManager> am;
    RobotDynamic robot_dynamic;
    double simulation_frequency;
    std::atomic_bool should_run{false};

    std::unordered_map<RobotIdentifier, SimulationTask> tasks;
    unsigned long simulation_id;
    std::optional<std::function<void(unsigned long, SimulationResult)>> result_callback;

    // runtime variables
    std::unordered_map<RobotIdentifier, Agent> agents;
    ThreadPool& thread_pool;
    std::optional<time::TimePoint> start_time;
    time::TimePoint current_time;
    std::map<RobotIdentifier, time::TimePoint> inactive_commands;

    logger::Logger logger{"LocalPlannerSimulator"};
    // id
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    static unsigned long last_simulation_id;
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    static std::mutex simulation_id_mtx;
};

/// @todo rotation vector callback
/// @todo result
/// @todo splitting
/// @todo real rotation callback
/// @todo thread pool
/// @todo trigger
/// @todo simulation manager

}  // namespace local_planner
}  // namespace luhsoccer