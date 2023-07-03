#include "robot_interface/robot_interface.hpp"
#include "config_provider/config_store_main.hpp"
#include "packets.hpp"

#include "connections/serial.hpp"
#include "connections/network.hpp"
#include "connections/simulation.hpp"

namespace luhsoccer::robot_interface {

class DefaultPacketBuilder : public PacketBuilder {
   public:
    void addMessage(const RobotCommandWrapper& /*cmd*/) override{
        // NOP
    };

    std::vector<RobotFeedbackWrapper> buildAndSend() override {
        // NOP
        return {};
    };
};

std::ostream& operator<<(std::ostream& os, const RobotConnection& source) {
    switch (source) {
        case RobotConnection::DISABLED:
            os << "Disabled";
            break;
        case RobotConnection::NETWORK:
            os << "Network";
            break;
        case RobotConnection::SERIAL:
            os << "Serial";
            break;
        case RobotConnection::SERIAL_LEGACY:
            os << "Serial (Legacy)";
            break;
        case RobotConnection::SIMULATION:
            os << "Simulation";
            break;
        case RobotConnection::SIMULATION_LEGACY:
            os << "Simulation (Legacy)";
    }
    return os;
}

RobotInterface::RobotInterface(simulation_interface::SimulationInterface& interface)
    : simulation_connection(std::make_unique<SimulationConnection>(interface)),
      serial_connection(std::make_unique<SerialConnection>()),
      network_connection(std::make_unique<NetworkConnection>()) {}

RobotInterface::~RobotInterface() = default;

void RobotInterface::setup() {
    // NOP
}

void RobotInterface::loop(std::atomic_bool& /*should_run*/) {
    const auto now = time::now();

    const auto builder = prepareSending();

    TeamColor color = TeamColor::BLUE;
    if (!config_provider::ConfigProvider::getConfigStore().game_config.is_blue) {
        color = TeamColor::YELLOW;
    }

    std::unique_lock lock(this->send_mutex);

    int counter = 0;

    for (auto it = this->active_robots.begin(); it != this->active_robots.end();) {
        const auto& element = *it;

        const auto wrapper = RobotCommandWrapper{static_cast<uint32_t>(element.first.id), color, element.second.first,
                                                 element.second.second};

        counter++;
        builder->addMessage(wrapper);

        this->robot_command_callbacks({element.first.id, element.second.second});
        bool remove = (now - element.second.first) > this->packet_repeat_duration;
        // Remove robot from sending list if there was no update for a certain time

        if (remove) {
            this->active_robots.erase(it++);
        } else {
            ++it;
        }
    }
    lock.unlock();

    if (counter != 0) {
        auto feedback_list = builder->buildAndSend();

        for (auto& feedback : feedback_list) {
            this->processFeedback(feedback);
        }
    }

    rate.sleep();
}

void RobotInterface::processFeedback(RobotFeedbackWrapper& feedback_wrapper) {
    bool is_blue = feedback_wrapper.color == TeamColor::BLUE;
    RobotIdentifier id(feedback_wrapper.id, Team::ALLY);
    if (is_blue == config_provider::ConfigProvider::getConfigStore().game_config.is_blue.val()) {
        std::lock_guard lock(this->feedback_mutex);
        if (!this->feedback_counters.contains(id)) {
            this->feedback_counters[id] = time::LoopStopwatch("RobotFeedbackFrequency", 100.0, 100);
        }

        this->feedback_counters[id].tik();

        // TODO always ally? this is correct behavior

        if (feedback_wrapper.feedback.telemetry_rf) {
            feedback_wrapper.feedback.telemetry_rf->frequency = this->feedback_counters[id].measuredFrequency();
        }
        this->robot_feedback_callbacks(std::make_pair(id, feedback_wrapper.feedback));
    }
}

void RobotInterface::stop() { this->serial_connection->stop(); };

void RobotInterface::replaceCommand(const RobotIdentifier& robot, RobotCommand command) {
    std::lock_guard lock(this->send_mutex);
    auto& element = this->active_robots[robot];
    element.first = time::now();
    element.second = std::move(command);
}

void RobotInterface::updateCommand(const RobotIdentifier& robot, const RobotCommand& command) {
    if (command.kick_command) {
        this->updateKickCommand(robot, command.kick_command.value());
    }
    if (command.move_command) {
        this->updateMoveCommand(robot, command.move_command.value());
    }
    if (command.dribbler_mode) {
        this->updateDribbler(robot, command.dribbler_mode.value());
    }

    std::lock_guard lock(this->send_mutex);
    this->active_robots[robot].first = time::now();
}

void RobotInterface::updateDribbler(const RobotIdentifier& robot, DribblerMode new_mode) {
    std::lock_guard lock(this->send_mutex);
    this->active_robots[robot].first = time::now();
    this->active_robots[robot].second.dribbler_mode = new_mode;
}

void RobotInterface::updateMoveCommand(const RobotIdentifier& robot, MoveCommand command) {
    std::lock_guard lock(this->send_mutex);
    this->active_robots[robot].first = time::now();
    this->active_robots[robot].second.move_command = std::move(command);
}

void RobotInterface::updateKickCommand(const RobotIdentifier& robot, KickCommand command) {
    std::lock_guard lock(this->send_mutex);
    this->active_robots[robot].first = time::now();
    this->active_robots[robot].second.kick_command = command;
}
void RobotInterface::clearKickCommand(const RobotIdentifier& robot) {
    std::lock_guard lock(this->send_mutex);
    this->active_robots[robot].first = time::now();
    this->active_robots[robot].second.kick_command = std::nullopt;
}

void RobotInterface::setConnectionType(const RobotConnection mode) {
    LOG_INFO(logger, "Change ssl interface mode from {} to {}", this->connection_type, mode);
    this->connection_type = mode;
    if (this->network_connection != nullptr) this->network_connection->updateBSHost();
}

std::unique_ptr<PacketBuilder> RobotInterface::prepareSending() {
    switch (this->connection_type) {
        case RobotConnection::SIMULATION:
            return std::make_unique<SimulationPacketBuilder>(*this->simulation_connection.get());
        case RobotConnection::SERIAL_LEGACY:
            return std::make_unique<SerialPacketBuilder>(*this->serial_connection.get());
        case RobotConnection::NETWORK:
            return std::make_unique<NetworkPacketBuilder>(*this->network_connection.get());
        case RobotConnection::DISABLED:
            return std::make_unique<DefaultPacketBuilder>();
        default:
            LOG_WARNING(logger, "No packet builder for connection type {}", this->connection_type);
            return std::make_unique<DefaultPacketBuilder>();
    }
}

}  // namespace luhsoccer::robot_interface