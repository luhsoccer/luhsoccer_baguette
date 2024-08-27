#pragma once

#include "core/module.hpp"
#include "core/robot_identifier.hpp"

#include "robot_control/robot_controller_state.hpp"
#include "robot_control/simulation_types.hpp"

namespace luhsoccer::transform {
class WorldModel;
}

namespace luhsoccer::robot_interface {
class RobotInterface;
}

namespace luhsoccer::marker {
class MarkerService;
}

namespace luhsoccer::robot_control {

class Skill;
struct TaskData;
class RobotController;
class MarkerAdapter;
class CooperationModule;
class SimulationManager;

class RobotControlModule : public BaguetteModule {
   public:
    RobotControlModule(const std::shared_ptr<const transform::WorldModel>& global_wm,
                       robot_interface::RobotInterface& robot_interface, marker::MarkerService& ms,
                       event_system::EventSystem& event_system);

    ~RobotControlModule() override;
    RobotControlModule(const RobotControlModule&) = delete;
    RobotControlModule& operator=(const RobotControlModule&) = delete;
    RobotControlModule(RobotControlModule&&) = delete;
    RobotControlModule& operator=(RobotControlModule&&) = delete;

    constexpr std::string_view moduleName() override { return "robot_control_module"; }

    void setup(event_system::EventSystem& event_system) override;

    void stop() override;

    bool setTask(const Skill* skill, const TaskData& td);

    const Skill* getSkill(const RobotIdentifier& robot) const;

    bool cancelTask(const RobotIdentifier& robot);

    [[nodiscard]] std::optional<RobotControllerState> getState(const RobotIdentifier& robot) const;

    void setModuleActive(bool active);

    bool setControllerActive(const RobotIdentifier& robot, bool active);

   private:
    void runSimulationOnTask(const Skill* skill, const TaskData& td);

    void checkControllerActiveState();

    void updateContinuousSimulations();

    std::unordered_map<RobotIdentifier, SimulationTask> continuous_simulation_tasks;
    std::mutex continuous_simulation_mtx;

    std::unordered_map<RobotIdentifier, std::unique_ptr<RobotController>> robot_controllers;

    std::shared_ptr<const transform::WorldModel> global_wm;
    robot_interface::RobotInterface& robot_interface;
    std::unique_ptr<MarkerAdapter> marker_adapter;
    std::unique_ptr<CooperationModule> coop_module;
    std::unique_ptr<SimulationManager> simulation_manager;

    bool last_controller_active_state{true};
    std::string last_active_controllers{"0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15"};
    logger::Logger logger;
};
}  // namespace luhsoccer::robot_control