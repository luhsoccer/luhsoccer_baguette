#include "robot_interface/robot_interface.hpp"
#include "robot_interface/events.hpp"
#include "config_provider/config_store_main.hpp"
#include "config/game_config.hpp"
#include "packets.hpp"

#include "connections/network.hpp"
#include "connections/simulation.hpp"

namespace luhsoccer::robot_interface {

class DefaultPacketBuilder : public PacketBuilder {
   public:
    void addMessage(const RobotCommandWrapper& /*cmd*/) override{
        // NOP
    };

    void buildAndSend() override{
        // NOP
    };
};

std::string_view format_as(const RobotConnection& type) {
    switch (type) {
        case RobotConnection::DISABLED:
            return "Disabled";
        case RobotConnection::NETWORK:
            return "Network";
        case RobotConnection::SERIAL:
            return "Serial";
        case RobotConnection::SIMULATION:
            return "Simulation";
    }
};

RobotInterface::RobotInterface(simulation_interface::SimulationInterface& interface) : interface(interface) {}

RobotInterface::~RobotInterface() = default;

void RobotInterface::setup(event_system::EventSystem& event_system) {
    simulation_connection = std::make_unique<SimulationConnection>(
        [&](RobotFeedbackWrapper wrapper) { this->processFeedback(wrapper, event_system); }, interface);
    network_connection = std::make_unique<NetworkConnection>(
        [&](RobotFeedbackWrapper wrapper) { this->processFeedback(wrapper, event_system); },
        event_system.getIoContext());

    event_system.registerEventHandler<event_system::TimerEvent100Hz>(
        [this](const event_system::EventContext<event_system::TimerEvent100Hz>& ctx) { this->update(ctx.system); });
}

void RobotInterface::update(event_system::EventSystem& event_system) {
    const auto now = time::now();

    const auto builder = prepareSending();

    TeamColor color = TeamColor::BLUE;
    if (!config_provider::ConfigProvider::getConfigStore().game_config.is_blue) {
        color = TeamColor::YELLOW;
    }

    auto active_robots_copy = [&]() {
        std::lock_guard lock(this->send_mutex);
        return this->active_robots;
    }();

    // TODO might have bad performance, because we're iterating through the same list twice
    for (const auto& robot : active_robots_copy) {
        event_system.fireEvent(RobotCommandSendEvent{robot.first, robot.second.second});
    }

    std::unique_lock lock(this->send_mutex);

    int counter = 0;

    // TODO we can't fireEvent here since we're in a lock and this can cause deadlocks with other events that are fired
    // in the same time for example the local planner might send updateCommand which causes a deadlock
    for (auto it = this->active_robots.begin(); it != this->active_robots.end();) {
        const auto& element = *it;

        const auto wrapper = RobotCommandWrapper{static_cast<uint32_t>(element.first.id), color, element.second.first,
                                                 element.second.second};

        counter++;
        builder->addMessage(wrapper);

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
        builder->buildAndSend();
    }
}

void RobotInterface::processFeedback(RobotFeedbackWrapper& feedback_wrapper, event_system::EventSystem& event_system) {
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
        event_system.fireEvent(RobotFeedbackReceivedEvent{id, feedback_wrapper.feedback});
    }
}

void RobotInterface::stop(){};

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
    logger.info("Change ssl interface mode from {} to {}", this->connection_type, mode);
    this->connection_type = mode;
    if (this->network_connection != nullptr) this->network_connection->updateBSHost();
}

std::unique_ptr<PacketBuilder> RobotInterface::prepareSending() {
    switch (this->connection_type) {
        case RobotConnection::SIMULATION:
            return std::make_unique<SimulationPacketBuilder>(*this->simulation_connection.get());
        case RobotConnection::NETWORK:
            return std::make_unique<NetworkPacketBuilder>(*this->network_connection.get());
        case RobotConnection::DISABLED:
            return std::make_unique<DefaultPacketBuilder>();
        default:
            logger.warning("No packet builder for connection type {}", this->connection_type);
            return std::make_unique<DefaultPacketBuilder>();
    }
}

}  // namespace luhsoccer::robot_interface
