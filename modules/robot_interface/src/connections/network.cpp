#include "connections/network.hpp"
#include "config_provider/config_store_main.hpp"
#include "config/game_config.hpp"
#include "config/robot_interface_config.hpp"
#include "core/visit.hpp"

#include <string>

namespace luhsoccer::robot_interface {

static RobotFeedbackWrapper convertFromBaseStation(const proto::basestation::FromBasestationPacket& packet) {
    RobotFeedbackWrapper wrapper{};
    wrapper.id = packet.id();
    // TODO later currently the robots don't know their own color so we always assume blue

    wrapper.color = packet.team_color() != config_provider::ConfigProvider::getConfigStore().game_config.is_blue
                        ? TeamColor::BLUE
                        : TeamColor::YELLOW;

    wrapper.feedback.has_ball = packet.has_ball();
    wrapper.feedback.position = std::nullopt;

    // TODO later time is not synced yet so use our local time
    wrapper.feedback.time_stamp = time::now();

    if (packet.has_local_velocity()) {
        wrapper.feedback.velocity = {packet.local_velocity().forward(), packet.local_velocity().left(),
                                     packet.local_velocity().counter_clockwise()};
    }

    wrapper.feedback.telemetry = RobotTelemetry();
    wrapper.feedback.telemetry->battery_voltage = packet.battery_voltage();
    wrapper.feedback.telemetry->capacitor_voltage = packet.kicker_voltage();
    wrapper.feedback.telemetry->light_barrier_working = false;
    wrapper.feedback.telemetry->quality = ConnectionQuality::GOOD;

    wrapper.feedback.telemetry_rf = RobotTelemetryRF();
    wrapper.feedback.telemetry_rf->rssi_robot = packet.rssi_robot();
    wrapper.feedback.telemetry_rf->rssi_base_station = packet.rssi_basestation();

    return wrapper;
}

NetworkPacketBuilder::NetworkPacketBuilder(NetworkConnection& connection) : connection(connection) {}

void NetworkPacketBuilder::addMessage(const RobotCommandWrapper& cmd) {
    luhsoccer::proto::basestation::ToBasestationPacket* packet = nullptr;

    if (!config_provider::ConfigProvider::getConfigStore().robot_interface_config.use_secondary_bs ||
        this->connection.bs1_robots.test(cmd.id)) {
        packet = this->wrapper.add_packets();
    } else {
        packet = this->second_bs_wrapper.add_packets();
    }

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
        if (cmd.cmd.kick_command->type == KickType::KICK) {
            kick_info->set_mode(proto::basestation::KickerMode::KICK);
        } else if (cmd.cmd.kick_command->type == KickType::CHIP) {
            kick_info->set_mode(proto::basestation::KickerMode::CHIP);
        }
        kick_info->set_relative(static_cast<float>(cmd.cmd.kick_command->velocity));

        if (cmd.cmd.kick_command->velocity > 0.0) {
            kick_info->set_charge_hint(proto::basestation::ChargeHint::DONT_CARE);
        } else {
            kick_info->set_charge_hint(proto::basestation::ChargeHint::DISCHARGE);
        }

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

void NetworkPacketBuilder::buildAndSend() { this->connection.sendPacket(this->wrapper, this->second_bs_wrapper); }

void NetworkConnection::setup() {
    const std::string& robots =
        config_provider::ConfigProvider::getConfigStore().robot_interface_config.first_bs_robots;
    auto split_string = [](const std::string& str, char delimiter) {
        std::bitset<16> result;
        std::stringstream ss = std::stringstream(str);
        std::string id;
        while (std::getline(ss, id, delimiter)) {
            result.set(std::stoi(id), true);
        }

        return result;
    };

    this->bs1_robots = split_string(robots, ',');
}

void NetworkConnection::updateBSHost() {
    const auto& config = config_provider::ConfigProvider::getConfigStore().robot_interface_config;

    if (this->endpoint)  //
        this->resolveHostname(*this->endpoint, config.network_hostname, config.network_port);

    if (this->second_endpoint)  //
        this->resolveHostname(*this->second_endpoint, config.secondary_network_hostname, config.network_port);
}

bool NetworkConnection::resolveHostname(asio::ip::udp::endpoint& target, const std::string& hostname,
                                        const uint32_t port, bool takefirst) {
    auto resolver = asio::ip::basic_resolver<asio::ip::udp>(this->ctx);

    asio::error_code ec;
    auto result = resolver.resolve(asio::ip::udp::v4(), hostname, std::to_string(port), ec);

    auto addr = target.address();
    for (const auto& entry : result) {
        // Ony update if the address changed
        if (!takefirst && entry.endpoint().address() == addr) continue;

        logger.info("Use basestation endpoint: {}:{}", entry.endpoint().address().to_string(), entry.endpoint().port());
        target = entry.endpoint();
        return true;
    }
    return false;
}

void NetworkConnection::sendPacket(const proto::basestation::ToBasestationWrapper& bs1_control,
                                   const proto::basestation::ToBasestationWrapper& bs2_control) {
    if (!socket.is_open()) {
        try {
            socket.open(asio::ip::udp::v4());
            receiveData();
        } catch (const asio::system_error& ex) {
            logger.error("Error while opening network socket: {} ({} {} {})", ex.what(), ex.code().category().name(),
                         ex.code().value(), ex.code().message());
        }
    }

    if (!this->endpoint) {
        const auto& config = config_provider::ConfigProvider::getConfigStore().robot_interface_config;

        this->endpoint = asio::ip::udp::endpoint();
        if (!this->resolveHostname(*this->endpoint, config.network_hostname, config.network_port, true))
            this->endpoint = std::nullopt;
    }

    if (!this->second_endpoint) {
        const auto& config = config_provider::ConfigProvider::getConfigStore().robot_interface_config;

        if (config.use_secondary_bs) {
            this->second_endpoint = asio::ip::udp::endpoint();
            if (!this->resolveHostname(*this->second_endpoint, config.secondary_network_hostname, config.network_port,
                                       true))
                this->second_endpoint = std::nullopt;
        }
    }

    if (this->endpoint || this->second_endpoint) {
        constexpr size_t BUFFER_SIZE = 8192;

        auto send_data = [&](const proto::basestation::ToBasestationWrapper& control,
                             asio::ip::udp::endpoint& target_endpoint) {
            auto buffer = control.SerializeAsString();

            try {
                socket.send_to(asio::buffer(buffer.c_str(), buffer.length()), target_endpoint);
                return true;
            } catch (const asio::system_error& ex) {
                logger.error("Error while sending network packet to bs ({}): {} ({} {} {})",
                             target_endpoint.address().to_string(), ex.what(), ex.code().category().name(),
                             ex.code().value(), ex.code().message());
            }

            return false;
        };

        bool data_sent = false;

        if (this->endpoint) {
            data_sent |= send_data(bs1_control, *endpoint);
        }

        if (this->second_endpoint) {
            data_sent |= send_data(bs2_control, *second_endpoint);
        }
    } else {
        logger.error("No endpoint for base station found");
    }
}

void NetworkConnection::receiveData() {
    socket.async_receive_from(
        asio::buffer(receive_buffer.data(), receive_buffer.size()), this->received_endpoint,
        [this](const asio::error_code& ec, std::size_t received_bytes) {
            std::lock_guard lock(this->mutex);

            if (ec) {
                logger.error("Error while receiving network packet: {}. Don't know how to recover from this.",
                             ec.message());
                return;
            }

            proto::basestation::FromBasestationWrapper basestation_wrapper;
            if (basestation_wrapper.ParseFromArray(receive_buffer.data(), static_cast<int>(received_bytes))) {
                uint32_t last_seq = this->sequence_ids[received_endpoint.address().to_v4().to_uint()];
                if (basestation_wrapper.seq_id() != last_seq) {
                    this->sequence_ids[received_endpoint.address().to_v4().to_uint()] = basestation_wrapper.seq_id();
                    for (const auto& packet : basestation_wrapper.packets()) {
                        on_feedback(convertFromBaseStation(packet));
                    }
                }
            } else {
                logger.error("Error while parsing feedback packet");
            }

            receiveData();
        });
}

void NetworkConnection::stop() {
#ifdef _WIN32
    this->socket.shutdown(asio::socket_base::shutdown_both);
#endif
    this->socket.close();
}

}  // namespace luhsoccer::robot_interface