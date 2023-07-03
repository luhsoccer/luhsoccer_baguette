

#pragma once

#include "module.hpp"
#include "local_planner/local_planner.hpp"
#include "local_planner/thread_pool.hpp"
#include "local_planner/simulation_manager.hpp"
namespace luhsoccer::local_planner {

class AvoidanceManager;
class LocalPlannerModule : public BaguetteModule {
   public:
    LocalPlannerModule(const std::shared_ptr<const transform::WorldModel>& wm,
                       robot_interface::RobotInterface& robot_interface, marker::MarkerService& ms);

    virtual ~LocalPlannerModule() = default;
    LocalPlannerModule(const LocalPlannerModule&) = delete;
    LocalPlannerModule& operator=(const LocalPlannerModule&) = delete;
    LocalPlannerModule(LocalPlannerModule&&) = delete;
    LocalPlannerModule& operator=(LocalPlannerModule&&) = delete;

    constexpr std::string_view moduleName() override { return "local_planner_module"; }

    void setup() override;

    void loop(std::atomic_bool& should_run) override;

    void stop() override;

    bool setTask(const Skill* skill, const TaskData& td);

    bool cancelTask(const RobotIdentifier& robot);

    std::optional<LocalPlanner::LocalPlannerState> getState(const RobotIdentifier& robot) const;

    void setState(bool online);

    bool setStateOfPlanner(RobotIdentifier robot, bool online);

    SimulationManager& getSimulationManager() { return simulation_manager; }

   private:
    void updateAvoidManagerModes();

    void visualizeTask(const Skill* skill, const TaskData& td);

    const config_provider::ConfigStore& cs;
    std::shared_ptr<const transform::WorldModel> wm;
    robot_interface::RobotInterface& robot_interface;

    marker::MarkerService& ms;
    std::unordered_map<RobotIdentifier, LocalPlanner> planners;
    ThreadPool thread_pool{};
    std::shared_ptr<AvoidanceManager> avoidance_manager;
    SimulationManager simulation_manager;
    logger::Logger logger;
};
}  // namespace luhsoccer::local_planner