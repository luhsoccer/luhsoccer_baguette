#pragma once

#include <utility>

#include "core/module.hpp"
#include "core/robot_identifier.hpp"
#include "robot_interface/robot_interface_types.hpp"
#include <mutex>

namespace luhsoccer::simulation_interface {
class SimulationInterface;
}

namespace luhsoccer::robot_interface {

/**
 * @brief This enum contains all possible states on how to communicate with the base station
 *
 */
enum class RobotConnection {
    /// Disable every communication with the base station
    DISABLED,
    /// Use the network protocol to communicate with the base station
    NETWORK,
    /// Use a serial connection to communicate with the base station
    SERIAL,
    /// Use only the simulation and not use a real base station
    SIMULATION,
};

std::string_view format_as(const RobotConnection& type);

class NetworkConnection;
class SimulationConnection;
class PacketBuilder;
struct RobotFeedbackWrapper;
class RobotInterface : public BaguetteModule {
   public:
    explicit RobotInterface(simulation_interface::SimulationInterface& interface);
    virtual ~RobotInterface();
    RobotInterface(const RobotInterface&) = delete;
    RobotInterface(RobotInterface&&) = delete;
    RobotInterface& operator=(const RobotInterface&) = delete;
    RobotInterface& operator=(RobotInterface&&) = delete;

    std::string_view moduleName() override { return "robot_interface"; }

    void setup(event_system::EventSystem& event_system) override;
    void update(event_system::EventSystem& event_system);
    void stop() override;

    void replaceCommand(const RobotIdentifier& robot, RobotCommand command);
    void updateCommand(const RobotIdentifier& robot, const RobotCommand& command);

    void updateDribbler(const RobotIdentifier& robot, DribblerMode new_mode);
    void updateMoveCommand(const RobotIdentifier& robot, MoveCommand command);
    void updateKickCommand(const RobotIdentifier& robot, KickCommand command);
    void clearKickCommand(const RobotIdentifier& robot);

    void setConnectionType(const RobotConnection mode);

    [[nodiscard]] RobotConnection getConnectionType() const { return this->connection_type; }

   private:
    void processFeedback(RobotFeedbackWrapper& feedback_wrapper, event_system::EventSystem& event_system);

    simulation_interface::SimulationInterface& interface;

    std::unique_ptr<PacketBuilder> prepareSending();

    logger::Logger logger{"robot_interface"};
    std::atomic<RobotConnection> connection_type{RobotConnection::DISABLED};

    std::unordered_map<RobotIdentifier, std::pair<time::TimePoint, RobotCommand>> active_robots;

    std::mutex feedback_mutex;
    std::unordered_map<RobotIdentifier, time::LoopStopwatch> feedback_counters;

    time::Duration packet_repeat_duration{1.0};
    std::mutex send_mutex;
    std::unique_ptr<NetworkConnection> network_connection;
    std::unique_ptr<SimulationConnection> simulation_connection;
};

}  // namespace luhsoccer::robot_interface