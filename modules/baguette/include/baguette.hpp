#pragma once

#include <memory>
#include <any>
#include <thread>
#include <list>

#include "core/module.hpp"

#include "simulation_interface/simulation_interface.hpp"
#include "robot_interface/robot_interface.hpp"
#include "skill_books/skill_library.hpp"
#include "ssl_interface/ssl_interface.hpp"
#include "game_data_provider/game_data_provider.hpp"
#include "marker_service/marker_service.hpp"
#include "luhviz/luhviz.hpp"
#include "config/config_store.hpp"
#include "config_provider/config_store_main.hpp"
#include "core/module.hpp"
#include "skill_books/skill_tester.hpp"
#include "scenario/scenario_executor.hpp"
#include "task_manager/task_manager.hpp"
#include "event_system/event_system.hpp"
#include "software_manager/software_manager.hpp"
#include "vision_processor/vision_processor.hpp"
#include "robot_control/robot_control_module.hpp"

// #define ENABLE_STATIC_SKILL_TESTER

namespace luhsoccer::baguette {

class TheBaguette {
   public:
    TheBaguette() { TheBaguette::instances.push_back(this); }
    ~TheBaguette() {
        logger.info("Destroying baguette instance");
        if (!this->exited) {
            logger.error("Destroying baguette instance which has not stopped yet. This can lead to dangerous errors.");
        }
        auto it = std::find(TheBaguette::instances.begin(), TheBaguette::instances.end(), this);
        if (it != TheBaguette::instances.end()) {
            (*it)->stop();
            TheBaguette::instances.erase(it);
        }
    }
    TheBaguette(const TheBaguette&) = delete;
    TheBaguette(const TheBaguette&&) = delete;
    TheBaguette& operator=(const TheBaguette&) = delete;
    TheBaguette& operator=(const TheBaguette&&) = delete;
    /**
     * @brief Load and starts all modules into their own threads
     *
     */
    void load(bool install_signal_handler = true);

    /**
     * @brief Runs all threads until the program should exit
     *
     */
    void run();

    /**
     * @brief Stops all modules and exists
     *
     */
    void stop();

    /**
     * @brief Checks if this instance is still running or has exited
     *
     * @return true
     * @return false
     */
    [[nodiscard]] bool isRunning() const { return should_run; };

   public:
    logger::Logger logger{"baguette"};
    std::vector<std::reference_wrapper<BaguetteModule>> modules;
    std::vector<std::thread> running_threads;
    std::atomic_bool started{false};
    std::atomic_bool should_run{false};
    std::atomic_bool exited{true};
    std::atomic_bool stopping{false};
    std::atomic_bool standalone{true};
    event_system::EventSystem event_system;

    // this is necessary here, since it's the only way to access the instances from the signal handler
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
    static std::vector<TheBaguette*> instances;
    static void signalHandler(int signal);

    template <typename T, typename... Args>
    std::unique_ptr<T> addModule(Args&&... args) {
        auto ptr = std::make_unique<T>(std::forward<Args>(args)...);
        modules.push_back(*ptr);
        return ptr;
    }

    // Special case. Should only be used for luhviz since luhviz can block the main thread
    template <typename T, typename... Args>
    std::unique_ptr<T> addLuhvizModule(Args&&... args) {
        auto ptr = std::make_unique<T>(std::forward<Args>(args)...);
        ptr->is_blocking = true;
        modules.push_back(*ptr);
        return ptr;
    }

   public:
    // Heap allocated modules. Here is the place to add new modules to the application
    std::unique_ptr<software_manager::SoftwareManager> software_manager =
        addModule<software_manager::SoftwareManager>();
    std::unique_ptr<marker::MarkerService> marker_service = addModule<marker::MarkerService>();
    std::unique_ptr<simulation_interface::SimulationInterface> simulation_interface =
        addModule<simulation_interface::SimulationInterface>(event_system);
    std::unique_ptr<ssl_interface::SSLInterface> ssl_interface =
        addModule<ssl_interface::SSLInterface>(*simulation_interface, event_system, *marker_service);
    std::unique_ptr<robot_interface::RobotInterface> robot_interface =
        addModule<robot_interface::RobotInterface>(*simulation_interface);
    std::unique_ptr<game_data_provider::GameDataProvider> game_data_provider =
        addModule<game_data_provider::GameDataProvider>(*marker_service, event_system);
    std::unique_ptr<vision_processor::VisionProcessor> vision_processor =
        addModule<vision_processor::VisionProcessor>();

    skills::SkillLibrary skill_lib{};

    std::unique_ptr<robot_control::RobotControlModule> robot_control_module =
        addModule<robot_control::RobotControlModule>(game_data_provider->getWorldModel(), *robot_interface,
                                                     *marker_service, event_system);

    std::unique_ptr<task_manager::TaskManager> task_manager =
        addModule<task_manager::TaskManager>(*game_data_provider, skill_lib, *robot_control_module);

#ifdef ENABLE_STATIC_SKILL_TESTER
    std::unique_ptr<skills::SkillTester> skill_tester =
        addModule<skills::SkillTester>(skill_lib, *local_planner, game_data_provider->getWorldModel());
#endif

    std::unique_ptr<scenario::ScenarioExecutor> scenario_executer =
        addModule<scenario::ScenarioExecutor>(game_data_provider->getWorldModel(), *simulation_interface,
                                              *robot_control_module, *marker_service, skill_lib, *robot_interface);

#ifndef DISABLE_LUHVIZ  // Disable luhviz when compile defintion is set
    std::unique_ptr<luhviz::LuhvizMain> luhviz = addLuhvizModule<luhviz::LuhvizMain>(
        *software_manager, *marker_service, *ssl_interface, *simulation_interface, *game_data_provider,
        *robot_control_module, skill_lib, *robot_interface, *scenario_executer);
#endif
};

}  // namespace luhsoccer::baguette