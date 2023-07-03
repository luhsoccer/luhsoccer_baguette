#pragma once

#include <map>
#include <mutex>
#include "module.hpp"
#include "logger/logger.hpp"
#include "simulation_interface/simulation_connector.hpp"
#include "robot_identifier.hpp"
#include <Eigen/Geometry>

namespace luhsoccer::simulation_interface {

using namespace simulation;

/**
 * @brief The simulation is a utility class to connect to various simulations.
 *
 * The test certain things without real robots, there are simulations. These speak a common protocol, which is
 * accessible via this class.
 * The SimulationInterfaces manages the connections to the various connectors, which might connect to an internal or
 * external simulation.
 * The exact type of the connection is defined in other classes, the "Connectors". The main task for this class is to
 * manage the active simulation and handle start and stop of connectors when a switch of the connector is requested.
 */
class SimulationInterface : public BaguetteModule {
   public:
    SimulationInterface() = default;
    SimulationInterface(const SimulationInterface&) = delete;
    SimulationInterface(SimulationInterface&&) = delete;
    SimulationInterface& operator=(const SimulationInterface&) = delete;
    SimulationInterface& operator=(SimulationInterface&&) = delete;
    virtual ~SimulationInterface() = default;

    /// Returns the name of this module
    std::string_view moduleName() override { return "simulation_interface"; }

    /// The setup function for this module
    void setup() override;

    /// The loop function for this module
    void loop(std::atomic_bool& should_run) override;

    /// The stop function for this module
    void stop() override;

    /**
     * @brief Sets the callback of the simulation vision output. This will always return the vision data from the
     * current active connector
     *
     * @param callback the new callback
     */
    void setVisionOutput(VisionOutputCallback callback) { this->packet_sink = std::move(callback); }

    /**
     * @brief Sets the callback of the simulation robot feedback output. This will always return the vision data from
     * the current active connector
     *
     * @param callback the new callback
     */
    void setRobotOutput(RobotOutputCallback callback) { this->robot_sink = std::move(callback); }

    /**
     * @brief Sets the callback of the simulation feedback output. This will always return the vision data from
     * the current active connector
     *
     * @param callback the new callback
     */
    void setSimulationOutput(SimulationOutputCallback callback) { this->simulation_sink = std::move(callback); }

    /**
     * @brief Switch the active connector
     *
     * This will switch the active connector, which will cause the old connector to disconnect and stop and the new
     * connector to load and connector.
     * There is no guarantee that the connector will hold their state after switching to a new type.
     *
     * @param type the type of the new connector
     */
    void switchConnector(const SimulationConnectorType type);

    /**
     * @brief Gets the current connector type
     *
     * @return SimulationConnectorType the current connector
     */
    [[nodiscard]] SimulationConnectorType getConnector() const;

    /**
     * @brief Sends a command to the robots in the simulation.
     *
     * This will send a command to the robots in the simulation. Based on the simulation their may be different effects
     * using the command. Will always forward to command to the active connector.
     *
     * @param control The control message
     * @param color Which team to send to
     */
    void sendRobotCommand(const RobotControl& control, const TeamColor color);

    /**
     * @brief Sends a command to the robots in the simulation.
     *
     * This will send a command to the simulation. Based on the simulation their may be different effects
     * using the command. Will always forward to command to the active connector.
     *
     * @param control The control message
     */
    void sendSimulationCommand(const LuhsoccerSimulatorControl& control);

    /**
     * @brief Teleport a robot in the simulation.
     *
     * @param which the id of the robot
     * @param team the color of the team
     * @param target the target position
     * @param velocity the target velocity
     * @param present if false the robot is removed from the simulation
     */
    void teleportRobot(unsigned int which, TeamColor team, const Eigen::Affine2d& target,
                       const Eigen::Vector3d& velocity = {0.0, 0.0, 0.0}, bool present = true);

    /**
     * @brief Teleport the ball in the simulation.
     *
     * @param target the target position
     * @param velocity the target velocity
     */
    void teleportBall(const Eigen::Affine2d& target, const Eigen::Vector3d& velocity = {0.0, 0.0, 0.0});

   private:
    VisionOutputCallback packet_sink = [](auto&&) {};
    RobotOutputCallback robot_sink = [](auto&&, auto&&) {};
    SimulationOutputCallback simulation_sink = [](auto&&) {};
    std::unordered_map<SimulationConnectorType, std::unique_ptr<SimulationConnector>> connectors{};
    mutable std::mutex connector_mutex{};
    SimulationConnectorType active_connector{SimulationConnectorType::NONE};
    logger::Logger logger{"simulation-interface"};

    template <typename T, typename... Args>
    void addConnector(Args&&... args) {
        std::unique_ptr<T> simulation =
            std::make_unique<T>(packet_sink, robot_sink, simulation_sink, std::forward<Args>(args)...);
        LOG_DEBUG(logger, "Added simulation connector: {}", simulation->type());
        connectors[simulation->type()] = std::move(simulation);
    }
};

}  // namespace luhsoccer::simulation_interface