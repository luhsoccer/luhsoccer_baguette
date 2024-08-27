#pragma once

#include <string_view>
#include <utility>
#include <functional>
#include "time/time.hpp"
#include "core/common_types.hpp"

class SimulationSyncResponse;

namespace luhsoccer::proto {
class RobotFeedback;
class RobotControl;
class LuhsoccerSimulatorControl;
}  // namespace luhsoccer::proto

namespace luhsoccer::proto::ssl_vision {
class SSL_WrapperPacket;
}

namespace luhsoccer::simulation {

using namespace proto;

using VisionOutputCallback = std::function<void(const ssl_vision::SSL_WrapperPacket&)>;
using RobotOutputCallback = std::function<void(const RobotFeedback&, const TeamColor)>;
using SimulationOutputCallback = std::function<void(const SimulationSyncResponse&)>;

enum class SimulationConnectorType { NONE, TEST_SIMULATION, ERFORCE_SIMULATION, ER_SIM };

std::string_view format_as(const SimulationConnectorType& type);

class SimulationConnector {
   public:
    SimulationConnector(VisionOutputCallback& vision_output, RobotOutputCallback& robot_feedback_output,
                        SimulationOutputCallback& simulation_feedback_output)
        : vision_output(vision_output),
          robot_feedback_output(robot_feedback_output),
          simulation_feedback_output(simulation_feedback_output) {}
    SimulationConnector(const SimulationConnector&) = delete;
    SimulationConnector(SimulationConnector&&) = delete;
    SimulationConnector& operator=(const SimulationConnector&) = delete;
    SimulationConnector& operator=(SimulationConnector&&) = delete;
    virtual ~SimulationConnector() = default;

    /**
     * @brief Returns the type of the connector
     *
     * @return std::string_view
     */
    [[nodiscard]] virtual SimulationConnectorType type() const = 0;

    /**
     * @brief Returns the current time of the simulation.
     *
     * @return time::TimePoint
     */
    [[nodiscard]] virtual time::TimePoint getSimulationTime() { return time::now(); }

    /**
     * @brief Loads the simulation. Will be executed each time the simulation connector becomes active again
     *
     */
    virtual void load(){};

    /**
     * @brief Stops the simulation. Will be executed each time the simulation connector switches.
     *
     */
    virtual void stop(){};

    /**
     * @brief This method will be called when there are new robot commands available.
     *
     * @param control
     * @param color
     */
    virtual void onRobotCommand(const RobotControl& /*control*/, const TeamColor /*color*/){/*nop by default*/};

    /**
     * @brief This method will be called when there are simulation commands available.
     *
     * @param control
     */
    virtual void onSimulationCommand(const LuhsoccerSimulatorControl& /*control*/){/*nop by default*/};

   protected:
    // The ouput for the vision data. Should be used to output the current position of the objects.
    VisionOutputCallback& vision_output;
    // The output for the robot data. Should be used to output information from the robots.
    RobotOutputCallback& robot_feedback_output;
    // The output for the simulation data. Should be used to output general information about the simulation
    SimulationOutputCallback& simulation_feedback_output;
};

}  // namespace luhsoccer::simulation