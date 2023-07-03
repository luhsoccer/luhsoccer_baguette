#include "connections/network.hpp"
#include <config_provider/config_store_main.hpp>
#include <visit.hpp>

namespace luhsoccer::robot_interface {

NetworkPacketBuilder::NetworkPacketBuilder(NetworkConnection& connection) : connection(connection) {}

void NetworkPacketBuilder::addMessage(const RobotCommandWrapper& cmd) {
    auto packet = this->wrapper.add_packets();
    packet->set_id(cmd.id);

    if (cmd.color == TeamColor::BLUE) {
        packet->set_team_color(proto::basestation::TeamColor::BLUE);
    } else if (cmd.color == TeamColor::YELLOW) {
        packet->set_team_color(proto::basestation::TeamColor::YELLOW);
    }

    auto dribbler_info = packet->mutable_dribbler_info();
    if (cmd.cmd.dribbler_mode.has_value()) {
        if (cmd.cmd.dribbler_mode == DribblerMode::OFF) {
            dribbler_info->set_tristate_mode(proto::basestation::TristateDribblerMode::OFF);
        } else if (cmd.cmd.dribbler_mode == DribblerMode::HIGH) {
            dribbler_info->set_tristate_mode(proto::basestation::TristateDribblerMode::FULL);
        } else if (cmd.cmd.dribbler_mode == DribblerMode::LOW) {
            dribbler_info->set_tristate_mode(proto::basestation::TristateDribblerMode::HALF);
        }
    } else {
        dribbler_info->set_tristate_mode(proto::basestation::TristateDribblerMode::OFF);
    }

    auto kick_info = packet->mutable_kicker_info();
    if (cmd.cmd.kick_command.has_value()) {
        kick_info->set_mode(proto::basestation::KickerMode::KICK);
        kick_info->set_charge_hint(proto::basestation::ChargeHint::DONT_CARE);
        kick_info->set_relative(static_cast<float>(cmd.cmd.kick_command->kick_velocity));
    } else {
        kick_info->set_charge_hint(proto::basestation::ChargeHint::DISCHARGE);
        kick_info->set_relative(0.0);
    }

    if (cmd.cmd.move_command.has_value()) {
        auto visitor = overload{[&](const RobotPositionControl& /*control*/) {
                                    throw std::runtime_error("RobotPositionControl not implemented");
                                },
                                [&](const RobotVelocityControl& control) {
                                    auto local_vel = packet->mutable_local_velocity();
                                    local_vel->set_forward(static_cast<float>(control.desired_velocity.x()));
                                    local_vel->set_left(static_cast<float>(control.desired_velocity.y()));
                                    local_vel->set_counter_clockwise(static_cast<float>(control.desired_velocity.z()));
                                }};

        std::visit(visitor, cmd.cmd.move_command.value());
    } else {
        auto local_vel = packet->mutable_local_velocity();
        local_vel->set_forward(0.0f);
        local_vel->set_left(0.0f);
        local_vel->set_counter_clockwise(0.0f);
    }
}

std::vector<RobotFeedbackWrapper> NetworkPacketBuilder::buildAndSend() {
    auto feedbacks = this->connection.sendPacket(this->wrapper);

    std::vector<RobotFeedbackWrapper> result;

    if (feedbacks) {
        for (const auto& feedback : feedbacks.value().packets()) {
            RobotFeedbackWrapper wrapper{};
            wrapper.id = feedback.id();
            // TODO later currently the robots don't know their own color so we always assume blue
            wrapper.color = config_provider::ConfigProvider::getConfigStore().game_config.is_blue ? TeamColor::BLUE
                                                                                                  : TeamColor::YELLOW;

            wrapper.feedback.has_ball = feedback.has_ball();
            wrapper.feedback.position = std::nullopt;
            // TODO later time is not synced yet so use our local time
            wrapper.feedback.time_stamp = time::now();

            if (feedback.has_local_velocity()) {
                wrapper.feedback.velocity = {feedback.local_velocity().forward(), feedback.local_velocity().left(),
                                             feedback.local_velocity().counter_clockwise()};
            }

            // TODO these value are not very useful we should change them in future
            wrapper.feedback.telemetry = RobotTelemetry();
            wrapper.feedback.telemetry->battery_voltage = feedback.battery_voltage();
            wrapper.feedback.telemetry->capacitor_voltage = feedback.kicker_voltage();
            wrapper.feedback.telemetry->light_barrier_working = false;
            wrapper.feedback.telemetry->quality = ConnectionQuality::GOOD;

            wrapper.feedback.telemetry_rf = RobotTelemetryRF();
            wrapper.feedback.telemetry_rf->rssi_robot = feedback.rssi_robot();
            wrapper.feedback.telemetry_rf->rssi_base_station = feedback.rssi_basestation();

            result.push_back(wrapper);
        }
    }

    return result;
}

void NetworkConnection::setup() {}

void NetworkConnection::updateBSHost() {
    if (endpoint) {
        const auto current_ip = this->endpoint->address();

        auto resolver = asio::ip::basic_resolver<asio::ip::udp>(this->context);

        asio::error_code ec;
        auto result = resolver.resolve(
            asio::ip::udp::v4(),
            config_provider::ConfigProvider::getConfigStore().robot_interface_config.network_hostname.val(),
            std::to_string(config_provider::ConfigProvider::getConfigStore().robot_interface_config.network_port.val()),
            ec);

        for (const auto& entry : result) {
            if (entry.endpoint().address() != current_ip) {
                this->endpoint = entry.endpoint();
                break;
            }
        }
    }
}

std::optional<proto::basestation::FromBasestationWrapper> NetworkConnection::sendPacket(
    const proto::basestation::ToBasestationWrapper& control) {
    if (this->context.stopped()) {
        return std::nullopt;
    }

    if (!socket.is_open()) {
        try {
            socket.open(asio::ip::udp::v4());
        } catch (const asio::system_error& ex) {
            LOG_ERROR(logger, "Error while opening network socket: {} ({})", ex.what(), ex.code());
        }
    }

    if (!endpoint) {
        auto resolver = asio::ip::basic_resolver<asio::ip::udp>(this->context);

        asio::error_code ec;
        auto result = resolver.resolve(
            asio::ip::udp::v4(),
            config_provider::ConfigProvider::getConfigStore().robot_interface_config.network_hostname.val(),
            std::to_string(config_provider::ConfigProvider::getConfigStore().robot_interface_config.network_port.val()),
            ec);

        for (const auto& endpoint : result) {
            LOG_INFO(logger, "Use base station endpoint at {}:{}", endpoint.endpoint().address(),
                     endpoint.endpoint().port());
            this->endpoint = endpoint.endpoint();
            break;
        }
    }

    if (endpoint) {
        auto buffer = control.SerializeAsString();

        try {
            socket.send_to(asio::buffer(buffer.c_str(), buffer.length()), endpoint.value());
        } catch (const asio::system_error& ex) {
            LOG_ERROR(logger, "Error while sending network packet: {} ({})", ex.what(), ex.code());
        }

        static time::LoopStopwatch stopwatch("NetworkConnection::sendPacket", 100.0, 100);

        std::optional<proto::basestation::FromBasestationWrapper> feedback_packet;
        while (socket.available()) {
            constexpr size_t BUFFER_SIZE = 8192;
            std::array<unsigned char, BUFFER_SIZE> receive_buffer{};
            size_t received_bytes = socket.receive(asio::buffer(receive_buffer, BUFFER_SIZE));

            feedback_packet.emplace();
            if (!feedback_packet.value().ParseFromArray(receive_buffer.data(), static_cast<int>(received_bytes))) {
                LOG_ERROR(logger, "Error while parsing feedback packet");
                return std::nullopt;
            } else {
                this->last_sequence_number = feedback_packet->seq_id();
            }
        }
        return feedback_packet;
    } else {
        LOG_ERROR(logger, "No endpoint for base station found");
    }

    return std::nullopt;
}

void NetworkConnection::stop() {
#ifdef _WIN32
    this->socket.shutdown(asio::socket_base::shutdown_both);
#endif
    this->socket.close();
    this->context.stop();
}

}  // namespace luhsoccer::robot_interface
