#pragma once

#include <utility>

#include "module.hpp"
#include "robot_identifier.hpp"
#include "robot_interface/callback_handler.hpp"
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
    /// Use a serial connection to communicate with the old base station. Legacy mode.
    SERIAL_LEGACY,
    /// Use only the simulation and not use a real base station
    SIMULATION,
    /// Use only the simulation in a legacy mode.
    SIMULATION_LEGACY,
};

std::ostream& operator<<(std::ostream& os, const RobotConnection& source);

class SerialConnection;
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

    void addCallback(const CallbackList<std::pair<RobotIdentifier, RobotFeedback>>::CallbackPtrBase& callback) {
        robot_feedback_callbacks.registerCallback(callback);
    };

    void removeCallback(const CallbackList<std::pair<RobotIdentifier, RobotFeedback>>::CallbackPtrBase& callback) {
        robot_feedback_callbacks.deregisterCallback(callback);
    }

    void addCommandCallback(const CallbackList<std::pair<uint32_t, RobotCommand>>::CallbackPtrBase& callback) {
        robot_command_callbacks.registerCallback(callback);
    };

    void removeCommandCallback(const CallbackList<std::pair<uint32_t, RobotCommand>>::CallbackPtrBase& callback) {
        robot_command_callbacks.deregisterCallback(callback);
    }

    void setup() override;
    void loop(std::atomic_bool& should_run) override;
    void stop() override;

    void replaceCommand(const RobotIdentifier& robot, RobotCommand command);
    void updateCommand(const RobotIdentifier& robot, const RobotCommand& command);

    void updateDribbler(const RobotIdentifier& robot, DribblerMode new_mode);
    void updateMoveCommand(const RobotIdentifier& robot, MoveCommand command);
    void updateKickCommand(const RobotIdentifier& robot, KickCommand command);
    void clearKickCommand(const RobotIdentifier& robot);

    void send(std::vector<RobotCommand> commands);

    void setConnectionType(const RobotConnection mode);

    [[nodiscard]] RobotConnection getConnectionType() const { return this->connection_type; }

   private:
    void processFeedback(RobotFeedbackWrapper& feedback_wrapper);

    std::unique_ptr<PacketBuilder> prepareSending();

    logger::Logger logger{"robot_interface"};
    RobotConnection connection_type{RobotConnection::DISABLED};
    CallbackList<std::pair<RobotIdentifier, RobotFeedback>> robot_feedback_callbacks;
    CallbackList<std::pair<uint32_t, RobotCommand>> robot_command_callbacks;

    std::unordered_map<RobotIdentifier, std::pair<time::TimePoint, RobotCommand>> active_robots;

    std::mutex feedback_mutex;
    std::unordered_map<RobotIdentifier, time::LoopStopwatch> feedback_counters;

    time::Rate rate{100.0, "RobotInterface"};
    time::Duration packet_repeat_duration{1.0};
    std::mutex send_mutex;
    std::unique_ptr<SerialConnection> serial_connection;
    std::unique_ptr<NetworkConnection> network_connection;
    std::unique_ptr<SimulationConnection> simulation_connection;
};

}  // namespace luhsoccer::robot_interface