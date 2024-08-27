#pragma once

#include <memory>
#include <string>
#include "core/module.hpp"

namespace luhsoccer::marker {
class MarkerService;
}

namespace luhsoccer::ssl_interface {
class SSLInterface;
}

namespace luhsoccer::simulation_interface {
class SimulationInterface;
}

namespace luhsoccer::game_data_provider {
class GameDataProvider;
}

namespace luhsoccer::robot_control {
class RobotControlModule;
}

namespace luhsoccer::skills {
class SkillLibrary;
}

namespace luhsoccer::robot_interface {
class RobotInterface;
}

namespace luhsoccer::scenario {
class ScenarioExecutor;
}

namespace luhsoccer::software_manager {
class SoftwareManager;
}

namespace luhsoccer::luhviz {

class LuhvizMain : public BaguetteModule {
   public:
    virtual ~LuhvizMain();
    LuhvizMain(LuhvizMain&&) = delete;
    LuhvizMain(LuhvizMain&) = delete;
    LuhvizMain& operator=(LuhvizMain&&) = delete;
    LuhvizMain& operator=(LuhvizMain&) = delete;

    LuhvizMain(software_manager::SoftwareManager& sm, marker::MarkerService& ms, ssl_interface::SSLInterface& ssl,
               simulation_interface::SimulationInterface& sim, game_data_provider::GameDataProvider& gdp,
               robot_control::RobotControlModule& robot_control, skills::SkillLibrary& skill_lib,
               robot_interface::RobotInterface& robot_interface, scenario::ScenarioExecutor& scenario_executor);

    void setup() override;
    void loop(std::atomic_bool& should_run) override;
    void stop() override;
    std::string_view moduleName() override { return "luhviz"; }

   private:
    class LuhvizMainImpl;
    std::unique_ptr<LuhvizMainImpl> implementation;
};

}  // namespace luhsoccer::luhviz