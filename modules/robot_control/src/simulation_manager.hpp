#pragma once
#include "marker_service/marker.hpp"
#include "robot_control/simulation_types.hpp"

namespace luhsoccer::transform {
class WorldModel;
}
namespace luhsoccer::event_system {
class EventSystem;
}

namespace luhsoccer::robot_control {
class Simulator;
class MarkerAdapter;

class SimulationManager {
   public:
    SimulationManager(const std::shared_ptr<const transform::WorldModel>& global_wm,
                      event_system::EventSystem& event_system);

    void startSkillSimulation(const std::vector<SimulationTask>& tasks, const ResultCallback& result_callback,
                              const time::TimePoint& start_time = time::now(),
                              std::shared_ptr<const transform::WorldModel> wm = nullptr);

    SimulationResult runSyncSimulation(const std::vector<SimulationTask>& tasks,
                                       const time::TimePoint& start_time = time::now(),
                                       std::shared_ptr<const transform::WorldModel> wm = nullptr);

    void stopSimulations();
    void stopSimulation(unsigned long id);

   private:
    event_system::EventSystem& event_system;
    std::atomic_bool simulation_manager_active{true};
    std::shared_ptr<const transform::WorldModel> global_wm;

    mutable std::shared_mutex simulator_mtx;
    std::unordered_map<unsigned long, std::shared_ptr<Simulator>> simulators;
    unsigned long last_id{0};
};

struct SimulationVizOptions {
    // NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    struct MapVelocity {
        marker::Color fast_color = marker::Color::RED();
        marker::Color slow_color = marker::Color::BLUE();

        double max_velocity = 3.0;
        double min_velocity = 0.0;
    };

    std::variant<marker::Color, MapVelocity> color = MapVelocity{};

    double lifetime = 10.0;
    double thickness = 0.03;
    double sampling = 0.01;

    size_t id = 0;
    std::optional<std::string> ns = std::nullopt;

    bool show_time = false;
    marker::Color time_color = marker::Color::RED();
    double time_scaling = 1.5;

    // NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
};

void visualizeSimulationResult(unsigned long id, const SimulationResult& result, const MarkerAdapter& ma,
                               const SimulationVizOptions& options = SimulationVizOptions());

void visualizeSimulationTask(unsigned long id, const SimulationResult& result, const RobotIdentifier& robot,
                             const MarkerAdapter& ma, const SimulationVizOptions& options = SimulationVizOptions());

}  // namespace luhsoccer::robot_control