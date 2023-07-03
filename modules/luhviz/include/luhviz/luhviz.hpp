#pragma once

#include <memory>
#include <string>
#include "module.hpp"

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

namespace luhsoccer::local_planner {
class LocalPlannerModule;
}

namespace luhsoccer::skills {
class BodSkillBook;
}

namespace luhsoccer::robot_interface {
class RobotInterface;
}

namespace luhsoccer::scenario {
class ScenarioExecutor;
}

namespace luhsoccer::luhviz {

class LuhvizMain : public BaguetteModule {
   public:
    virtual ~LuhvizMain();
    LuhvizMain(LuhvizMain&&) = delete;
    LuhvizMain(LuhvizMain&) = delete;
    LuhvizMain& operator=(LuhvizMain&&) = delete;
    LuhvizMain& operator=(LuhvizMain&) = delete;

    LuhvizMain(marker::MarkerService& ms, ssl_interface::SSLInterface& ssl,
               simulation_interface::SimulationInterface& sim, game_data_provider::GameDataProvider& gdp,
               local_planner::LocalPlannerModule& local_planner, skills::BodSkillBook& skill_book,
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