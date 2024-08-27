#include "simulation_interface/simulation_connector.hpp"

namespace luhsoccer::simulation {
std::string_view format_as(const SimulationConnectorType& type) {
    switch (type) {
        case SimulationConnectorType::NONE:
            return "None";
        case SimulationConnectorType::TEST_SIMULATION:
            return "Test-Simulation";
        case SimulationConnectorType::ER_SIM:
            return "ErSim";
        case SimulationConnectorType::ERFORCE_SIMULATION:
            return "ErForce-Simulation";
    }
}
};  // namespace luhsoccer::simulation