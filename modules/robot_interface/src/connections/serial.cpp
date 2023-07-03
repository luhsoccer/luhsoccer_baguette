
#include "connections/serial.hpp"
#include "visit.hpp"
#include "config_provider/config_store_main.hpp"

namespace luhsoccer::robot_interface {

SerialPacketBuilder::SerialPacketBuilder(SerialConnection& connection) : connection(connection) {
    this->packets.reserve(6);
}

void SerialPacketBuilder::addMessage(const RobotCommandWrapper& cmd) {
    SerialToRobotPacket packet{};
    packet.packet_type = 1;
    packet.robot_id = cmd.id;

    if (cmd.cmd.dribbler_mode) {
        switch (cmd.cmd.dribbler_mode.value()) {
            case DribblerMode::OFF:
                packet.dribbler_strength = 0;
                break;
            case DribblerMode::LOW:
                packet.dribbler_strength = 1;
                break;
            case DribblerMode::HIGH:
                packet.dribbler_strength = 2;
                break;
        }
    }

    if (cmd.cmd.move_command) {
        auto visitor = overload{
            [&](const RobotPositionControl& /*control*/) {
                LOG_WARNING(logger::Logger("serial_connection"),
                            "Legacy serial connection can only handle velocity control");
            },
            [&](const RobotVelocityControl& control) {
                constexpr double MAX_VELOCITY = 10.0;
                constexpr double MAX_ROTATION = 4.0;
                packet.velocity_x =
                    static_cast<int>(control.desired_velocity.x() / MAX_VELOCITY * std::numeric_limits<int16_t>::max());
                packet.velocity_y =
                    static_cast<int>(control.desired_velocity.y() / MAX_VELOCITY * std::numeric_limits<int16_t>::max());
                packet.rotation =
                    static_cast<int>(control.desired_velocity.z() / MAX_ROTATION * std::numeric_limits<int16_t>::max());
            }};

        std::visit(visitor, cmd.cmd.move_command.value());
    }

    if (cmd.cmd.kick_command) {
        // const proto::KickCommand& kick_cmd = cmd.kick_command();
        if (cmd.cmd.kick_command->cap_voltage) {
            // NOLINTNEXTLINE: Legacy code
            packet.kicker_voltage = cmd.cmd.kick_command->cap_voltage.value() * 255.0 / 450.0;
            // NOLINTNEXTLINE: Legacy code
            packet.kicker_strength = 127;
        } else {
            LOG_WARNING(logger::Logger("serial_connection"), "Cap voltage is necessary for legacy communication");
        }
    }

    this->packets.push_back(packet);
};

std::vector<RobotFeedbackWrapper> SerialPacketBuilder::buildAndSend() {
    constexpr int MAX_ROBOTS_LEGACY = 6;
    if (packets.size() >= MAX_ROBOTS_LEGACY) {
        LOG_WARNING(logger::Logger("serial_connection"),
                    "Capping robots because count of robots is greater {}. Currently there are {} active robots",
                    MAX_ROBOTS_LEGACY, packets.size());
    }

    const size_t number_of_robots = std::min(packets.size(), static_cast<size_t>(MAX_ROBOTS_LEGACY));

    const size_t send_buffer_length = sizeof(SerialToRobotPacket) * number_of_robots + 3;
    std::string buffer;
    buffer.reserve(send_buffer_length);
    buffer += 'S';
    buffer += static_cast<char>(number_of_robots);

    size_t counter = 0;
    for (const auto& packet : this->packets) {
        // NOLINTBEGIN needed for legacy protocol
        std::array<char, 9> bytes{};
        bytes[0] = (packet.dribbler_strength << 6) | (packet.robot_id << 2) | (packet.packet_type);
        bytes[1] = (packet.velocity_x >> 0) & 0xFF;
        bytes[2] = (packet.velocity_x >> 8) & 0xFF;
        bytes[3] = (packet.velocity_y >> 0) & 0xFF;
        bytes[4] = (packet.velocity_y >> 8) & 0xFF;
        bytes[5] = (packet.rotation >> 0) & 0xFF;
        bytes[6] = (packet.rotation >> 8) & 0xFF;
        bytes[7] = packet.kicker_strength > 0 ? 255 : 0;
        bytes[8] = packet.kicker_voltage;
        // NOLINTEND

        const std::string cmd_buffer(bytes.data(), TO_ROBOT_PACKET_SIZE);
        buffer += cmd_buffer;
        counter++;
        if (counter > MAX_ROBOTS_LEGACY) {
            break;
        }
    }

    buffer += 'E';

    return this->connection.send(buffer);
}

void SerialConnection::stop() {
    if (this->port.isOpen()) {
        this->port.close();
    }
}

std::vector<RobotFeedbackWrapper> SerialConnection::send(const std::string& data) {
    if (!this->port.isOpen()) {
        std::string port_name = config_provider::ConfigProvider::getConfigStore().robot_interface_config.serial_port;
        if (port_name == "auto") {
            const auto ports = serial::list_ports();
            if (ports.empty()) {
                LOG_WARNING(logger, "Port discovery is set to 'auto' but no port was found!");
                std::this_thread::sleep_for(std::chrono::seconds(1));
                return {};
            }
            port_name = ports[0].port;
        }
        LOG_INFO(logger, "Opening port: {}", port_name);
        port.setPort(port_name);

        try {
            port.open();
            port.setDTR();
        } catch (serial::SerialException& e) {
            LOG_ERROR(logger, "Error while opening serial port: {}", e.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } catch (serial::IOException& e) {
            LOG_ERROR(logger, "Error while opening serial port: {}", e.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } catch (...) {
            LOG_ERROR(logger, "Error while opening serial port");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    std::vector<RobotFeedbackWrapper> robot_feedbacks;

    if (this->port.isOpen()) {
        try {
            size_t bytes = port.write(data);

            if (bytes != data.length()) {
                LOG_WARNING(logger, "Could not send all bytes. Buffer: {}, Send: {}", data.length(), bytes);
            }

            if (port.available()) {
                std::vector<uint8_t> buffer;
                buffer.reserve(port.available());
                auto bytes = port.read(buffer, buffer.capacity());

                auto iter = buffer.begin();

                while (iter != buffer.end()) {
                    if (*iter == 'S') {
                        FromRobotPacketBuffer packet_buffer{};
                        size_t index = 0;
                        iter++;
                        while (iter != buffer.end() && *iter != 'E') {
                            packet_buffer[index] = *iter;
                            if (index > packet_buffer.size()) {
                                break;
                            }
                            ++iter;
                            ++index;
                        }

                        robot_feedbacks.emplace_back(fromSerialPacket(packet_buffer));
                    }
                    ++iter;
                }
            }

        } catch (serial::SerialException& e) {
            LOG_ERROR(logger, "Error while r/w serial port: {}", e.what());
            port.close();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        } catch (serial::IOException& e) {
            LOG_ERROR(logger, "Error while r/w serial port: {}", e.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
            port.close();
        } catch (...) {
            LOG_ERROR(logger, "Error while r/w serial port");
            port.close();
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    return robot_feedbacks;
}

RobotFeedbackWrapper SerialConnection::fromSerialPacket(const FromRobotPacketBuffer& buffer) {
    // NOLINTBEGIN this is needed for the legacy protocol
    SerialFromRobotPacket packet;
    packet.packet_type = buffer[0] & 0b11;
    packet.robot_id = (buffer[0] >> 2) & 0b1111;
    packet.error = (buffer[0] >> 6) & 0b11;
    packet.battery_voltage = (buffer[1] & 0b1111111);
    packet.has_ball_in_dribbler = (buffer[1] >> 7) & 0b1;
    packet.kicker_voltage = buffer[2];
    packet.delta_x = buffer[3];
    packet.delta_y = buffer[4];
    packet.delta_rotation = buffer[5];
    // NOLINTEND

    RobotFeedbackWrapper wrapper{};
    wrapper.feedback.telemetry = RobotTelemetry{};
    wrapper.feedback.velocity = Eigen::Vector3d{0, 0, 0};
    wrapper.id = packet.robot_id;
    wrapper.color =
        config_provider::ConfigProvider::getConfigStore().game_config.is_blue ? TeamColor::BLUE : TeamColor::YELLOW;

    bool has_ball = packet.has_ball_in_dribbler;
    wrapper.feedback.has_ball = has_ball;
    wrapper.feedback.time_stamp = time::now();
    wrapper.feedback.velocity->x() = static_cast<float>(packet.delta_x) * (10.0f / 127.0f);
    wrapper.feedback.velocity->y() = static_cast<float>(packet.delta_y) * (10.0f / 127.0f);
    wrapper.feedback.velocity->z() = static_cast<float>(packet.delta_rotation) * (4.0f / 127.0f);
    wrapper.feedback.telemetry->battery_voltage =
        static_cast<float>(packet.battery_voltage) * ((27.0f - 16.2f) / 127.0f) + 16.2f;
    wrapper.feedback.telemetry->capacitor_voltage = static_cast<float>(packet.kicker_voltage) * (300.0f / 255.0f);
    wrapper.feedback.telemetry->light_barrier_working = true;
    wrapper.feedback.telemetry->quality = ConnectionQuality::GOOD;

    return wrapper;
}

}  // namespace luhsoccer::robot_interface