// Here is place for your own code
#include "robot_control/robot_control_module.hpp"
#include "config/game_config.hpp"
#include "config/robot_control_config.hpp"
#include "config/robot_control_visualization_config.hpp"
#include "marker_adapter.hpp"
#include "robot_controller.hpp"
#include "simulation_manager.hpp"

#include "robot_control/components/component_util.hpp"
#include "cooperation_module.hpp"
#include "robot_control/skills/skill.hpp"
#include "config_provider/config_store_main.hpp"

namespace luhsoccer::robot_control {
RobotControlModule::RobotControlModule(const std::shared_ptr<const transform::WorldModel>& global_wm,
                                       robot_interface::RobotInterface& robot_interface, marker::MarkerService& ms,
                                       event_system::EventSystem& event_system)
    : global_wm(global_wm),
      robot_interface(robot_interface),
      marker_adapter(std::make_unique<MarkerAdapter>(ms)),
      coop_module(std::make_unique<CooperationModule>()),
      simulation_manager(std::make_unique<SimulationManager>(global_wm, event_system)),
      logger("RobotControlModule") {}

RobotControlModule::~RobotControlModule() = default;

void RobotControlModule::setup(event_system::EventSystem& event_system) {
    std::vector<RobotIdentifier> robots = this->global_wm->getPossibleRobots<Team::ALLY>();

    for (const RobotIdentifier& robot : robots) {
        this->logger.debug("Creating robot controller for {} with frequency {:d}Hz", robot,
                           robotControlConfig().controllers_frequency.val());

        auto [it, success] =
            this->robot_controllers.emplace(std::piecewise_construct, std::forward_as_tuple(robot),
                                            std::forward_as_tuple(std::make_unique<RobotController>(
                                                this->global_wm, *this->marker_adapter, robot, *this->coop_module)));
        if (success) {
            it->second->startController(this->robot_interface, event_system,
                                        robotControlConfig().controllers_frequency);
        }
    }

    event_system.registerEventHandler<event_system::TimerEvent2Sec>(
        [this](event_system::EventContext<event_system::TimerEvent2Sec>) { this->checkControllerActiveState(); });
    event_system.registerEventHandler<event_system::TimerEvent10Hz>(
        [this](event_system::EventContext<event_system::TimerEvent10Hz>) { this->updateContinuousSimulations(); });
}

void RobotControlModule::stop() {
    for (auto& [robot, controller] : this->robot_controllers) {
        controller->cancelCurrentTask();
    }
}

bool RobotControlModule::setTask(const Skill* skill, const TaskData& td) {
    auto it = this->robot_controllers.find(td.robot);

    if (it == this->robot_controllers.end()) {
        this->logger.warning("No controller exists for requested {}. Task could not be setted!.", td.robot);
        return false;
    }

    bool success = it->second->setTask(skill, td);

    if (success && robotControlVisualizationConfig().simulate_tasks) {
        this->runSimulationOnTask(skill, td);
    }

    if (success && robotControlVisualizationConfig().simulate_tasks_continuously) {
        std::lock_guard lock(this->continuous_simulation_mtx);
        this->continuous_simulation_tasks.emplace(td.robot, SimulationTask(skill, td));
    }

    return success;
}
const Skill* RobotControlModule::getSkill(const RobotIdentifier& robot) const {
    auto controller = this->robot_controllers.find(robot);
    if (controller == this->robot_controllers.end()) return nullptr;
    return controller->second->getSkill();
}

bool RobotControlModule::cancelTask(const RobotIdentifier& robot) {
    auto it = this->robot_controllers.find(robot);

    if (it == this->robot_controllers.end()) {
        this->logger.warning("No controller exists for requested {}. Task could not be canceled!.", robot);
        return false;
    }

    bool success = it->second->cancelCurrentTask();

    if (success && robotControlVisualizationConfig().simulate_tasks_continuously) {
        std::lock_guard lock(this->continuous_simulation_mtx);
        this->continuous_simulation_tasks.erase(robot);
    }

    return success;
}

void RobotControlModule::updateContinuousSimulations() {
    std::lock_guard lock(this->continuous_simulation_mtx);

    std::erase_if(this->continuous_simulation_tasks, [this](const auto& task) {
        auto controller_it = this->robot_controllers.find(task.first);
        if (controller_it == this->robot_controllers.end()) return true;
        return controller_it->second->getState() != RobotControllerState::RUNNING;
    });

    for (auto& [robot, task] : this->continuous_simulation_tasks) {
        this->runSimulationOnTask(task.skill, task.task_data);
    }
}

std::optional<RobotControllerState> RobotControlModule::getState(const RobotIdentifier& robot) const {
    auto it = this->robot_controllers.find(robot);
    if (it == this->robot_controllers.end()) return std::nullopt;

    return it->second->getState();
}

void RobotControlModule::setModuleActive(bool online) {
    for (auto& [key, planner] : this->robot_controllers) {
        planner->setActive(online);
    }
}

bool RobotControlModule::setControllerActive(const RobotIdentifier& robot, bool online) {
    auto robot_it = this->robot_controllers.find(robot);
    if (robot_it == this->robot_controllers.end()) return false;
    robot_it->second->setActive(online);
    return true;
}

void RobotControlModule::checkControllerActiveState() {
    bool new_active_state = config_provider::ConfigProvider::getConfigStore().game_config.robot_controllers_active;
    std::string new_active_controllers_str =
        config_provider::ConfigProvider::getConfigStore().game_config.active_robot_controllers;

    if (new_active_state != this->last_controller_active_state) {
        this->setModuleActive(new_active_state);
        this->last_controller_active_state = new_active_state;
    }

    if (new_active_controllers_str.compare(this->last_active_controllers) != 0) {
        auto split_string = [](std::string string) {
            size_t pos = 0;
            std::vector<std::string> sub_strings;
            const std::string delimiter = ",";
            while ((pos = string.find(delimiter)) != std::string::npos) {
                std::string token = string.substr(0, pos);
                string.erase(0, pos + delimiter.length());
                sub_strings.push_back(token);
            }
            return sub_strings;
        };

        std::vector<std::string> active_controllers = split_string(new_active_controllers_str);

        for (auto& [key, planner] : this->robot_controllers) {
            bool controller_active = std::find(active_controllers.begin(), active_controllers.end(),
                                               std::to_string(key.id)) != active_controllers.end();
            planner->setActive(controller_active);
        }
        this->last_active_controllers = new_active_controllers_str;
    }
}

void RobotControlModule::runSimulationOnTask(const Skill* skill, const TaskData& td) {
    std::vector<SimulationTask> tasks;
    tasks.emplace_back(skill, td);
    time::TimePoint sim_start_time = time::now();
    auto result_callback = [this, sim_start_time](unsigned long id, const SimulationResult& result) {
        time::Duration task_duration = (result.end_time - result.start_time);
        time::Duration sim_duration = (time::now() - sim_start_time);
        constexpr int SEC_TO_MILLISEC = 1000;
        this->logger.debug("Simulation {:d} finished. Task took {:0.2f}s. Simulation took {:.0f}ms", id,
                           task_duration.asSec(), sim_duration.asSec() * SEC_TO_MILLISEC);

        SimulationVizOptions opts;
        opts.show_time = true;
        visualizeSimulationResult(id, result, *this->marker_adapter, opts);
    };

    this->simulation_manager->startSkillSimulation(tasks, result_callback);
}

}  // namespace luhsoccer::robot_control