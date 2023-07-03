#include "simulations/erforce_simulation_connector.hpp"
#include "luhsoccer_robot_interface.pb.h"
#include "luhsoccer_simulation_control.pb.h"
#include "ssl_vision_wrapper.pb.h"
#include <cstddef>
#include <cstdlib>
#include "config_provider/config_store_main.hpp"

namespace luhsoccer::simulation {

std::array<std::byte, 4> ErforceSimulationConnector::lengthToBytes(uint32_t length) {
    // clang-format off
    return std::array<std::byte, 4>{
        std::byte(length >> FORTH_BYTE & BYTE_MASK),
        std::byte(length >> THIRD_BYTE & BYTE_MASK),
        std::byte(length >> SECOND_BYTE & BYTE_MASK),
        std::byte(length >> FIRST_BYTE & BYTE_MASK)
    };
    // clang-format on
}

uint32_t ErforceSimulationConnector::bytesToLength(std::array<std::byte, sizeof(uint32_t)> bytes) {
    // clang-format off
    static_assert(sizeof(uint32_t) == 4, "Code only works with 4 byte uint32_t");
    return (std::to_integer<uint32_t>(bytes[0]) << FORTH_BYTE) |
           (std::to_integer<uint32_t>(bytes[1]) << THIRD_BYTE) |
           (std::to_integer<uint32_t>(bytes[2]) << SECOND_BYTE)|
           (std::to_integer<uint32_t>(bytes[3]) << FIRST_BYTE);
    // clang-format on
}

void ErforceSimulationConnector::handleSizedRead(ErforceSimulationConnector::SocketConnection& socket,
                                                 uint32_t length_to_read) {
    asio::async_read(socket.socket, asio::buffer(socket.data_buffer, length_to_read),
                     [this, &socket, length_to_read](const std::error_code& ec, std::size_t length) {
                         if (ec) {
                             LOG_WARNING(logger, "Got error code {} with message: {}", ec.value(), ec.message());
                             socket.connected = false;
                         } else {
                             if (length == length_to_read) {
                                 (this->*socket.callback)(socket, length_to_read);
                             } else {
                                 LOG_WARNING(logger, "Received wrong message size. Expected {} got {}", length_to_read,
                                             length);
                             }
                             handleHeaderRead(socket);
                         }
                     });
}

void ErforceSimulationConnector::handleHeaderRead(ErforceSimulationConnector::SocketConnection& socket) {
    asio::async_read(socket.socket, asio::buffer(socket.header_buffer),
                     [this, &socket](const std::error_code& ec, std::size_t length) {
                         if (ec) {
                             LOG_WARNING(logger, "Got error code {} with message: {}", ec.value(), ec.message());
                             socket.connected = false;
                         } else {
                             if (length == HEADER_BUFFER_SIZE) {
                                 handleSizedRead(socket, this->bytesToLength(socket.header_buffer));
                             } else {
                                 LOG_WARNING(logger, "Received wrong header size. Expected {} got {}",
                                             HEADER_BUFFER_SIZE, length);
                                 handleHeaderRead(socket);
                             }
                         }
                     });
}

void ErforceSimulationConnector::connectSocket(ErforceSimulationConnector::SocketConnection& socket) {
    resolver.async_resolve(
        socket.host, socket.port,
        [this, &socket](const std::error_code& ec, const asio::ip::basic_resolver_results<asio::ip::tcp>& endpoints) {
            if (ec) {
                LOG_WARNING(logger, "Could not resolve {}:{}: {} ({})", socket.host, socket.port, ec.message(),
                            ec.value());
            } else {
                if (socket.connecting) {
                    return;
                }
                socket.connecting = true;
                asio::async_connect(
                    socket.socket, endpoints,
                    [this, &socket](const std::error_code& ec, asio::ip::tcp::endpoint new_endpoint) {
                        socket.connecting = false;
                        socket.connected = !ec;
                        if (!ec) {
                            LOG_DEBUG(logger, "Connected to: {}", new_endpoint);
                            handleHeaderRead(socket);
                        } else {
                            LOG_WARNING(logger, "Could not connect to {}:{}. Error: {} ({})", socket.host, socket.port,
                                        ec.message(), ec.value());
                            socket.reconnect_timer.expires_after(std::chrono::seconds(3));
                            socket.reconnect_timer.async_wait([this, &socket](const std::error_code& ec) {
                                if (ec) {
                                    LOG_WARNING(logger, "Error while trying to reconnect: {} ({})", ec.message(),
                                                ec.value());
                                } else {
                                    connectSocket(socket);
                                }
                            });
                        }
                    });
            }
        });
}

void ErforceSimulationConnector::load() {
    this->connectSocket(vision_connection);
    this->connectSocket(simulation_connection);

    if (config_provider::ConfigProvider::getConfigStore().game_config.is_blue) {
        this->openRobotSocket(TeamColor::BLUE);
    } else {
        this->openRobotSocket(TeamColor::YELLOW);
    }

    this->context.restart();
};

void ErforceSimulationConnector::openRobotSocket(TeamColor color) {
    static std::mutex mutex{};
    std::lock_guard lock(mutex);
    LOG_DEBUG(logger, "Opening robot socket for color: {}", color);
    if (color == TeamColor::BLUE) {
        this->connectSocket(blue_robot_connection);
    } else {
        this->connectSocket(yellow_robot_connection);
    }
    this->current_controlling = color;
}

void ErforceSimulationConnector::closeRobotSocket(TeamColor color) {
    LOG_INFO(logger, "Closing robot socket for color: {}", color);
    if (color == TeamColor::BLUE) {
        if (this->blue_robot_connection.socket.is_open()) {
            this->blue_robot_connection.connected = false;
            this->blue_robot_connection.socket.close();
        }
    } else {
        if (this->yellow_robot_connection.socket.is_open()) {
            this->yellow_robot_connection.connected = false;
            this->yellow_robot_connection.socket.close();
        }
    }
}

void ErforceSimulationConnector::handleRobotRead(ErforceSimulationConnector::SocketConnection& socket,
                                                 uint32_t message_length) {
    RobotControlResponse robot_feedbacks;
    if (robot_feedbacks.ParseFromArray(socket.data_buffer.data(), static_cast<int>(message_length))) {
        handleSimulationError(robot_feedbacks);
        for (const auto& robot_feedback : robot_feedbacks.feedback()) {
            if (&socket == &blue_robot_connection) {
                robot_feedback_output(robot_feedback, TeamColor::BLUE);
            } else if (&socket == &yellow_robot_connection) {
                robot_feedback_output(robot_feedback, TeamColor::YELLOW);
            }
        }
    } else {
        LOG_WARNING(logger, "Received malformed robot control response packet");
    }
}

void ErforceSimulationConnector::handleVisionRead(ErforceSimulationConnector::SocketConnection& socket,
                                                  uint32_t message_length) {
    ssl_vision::SSL_WrapperPacket wrapper;
    if (wrapper.ParseFromArray(socket.data_buffer.data(), static_cast<int>(message_length))) {
        this->vision_output(wrapper);
    } else {
        LOG_WARNING(logger, "Received malformed vision packet");
    }
}

void ErforceSimulationConnector::handleSimulationRead(ErforceSimulationConnector::SocketConnection& socket,
                                                      uint32_t message_length) {
    LuhsoccerSimulatorFeedback simulation_feedback;
    if (simulation_feedback.ParseFromArray(socket.data_buffer.data(), static_cast<int>(message_length))) {
        handleSimulationError(simulation_feedback);

        if (simulation_feedback.has_sync_response()) {
            simulation_feedback_output(simulation_feedback.sync_response());
        }
    } else {
        LOG_WARNING(logger, "Received malformed simulation feedback packet");
    }
}

void ErforceSimulationConnector::onRobotCommand(const RobotControl& control,
                                                const TeamColor color) {  // TODO use team color
    auto do_work = [&](SocketConnection& connection) {
        if (!connection.connected) return;

        uint32_t len = control.ByteSizeLong();
        auto len_buffer = lengthToBytes(len);

        // Fill the first 4 byte of the buffer with the length
        std::copy(len_buffer.begin(), len_buffer.end(), this->robot_send_buffer.begin());
        // Write the message into the buffer, with the offset of the length buffer
        if (control.SerializeToArray(this->robot_send_buffer.data() + len_buffer.size(),
                                     static_cast<int>(this->robot_send_buffer.size() - len_buffer.size()))) {
            asio::async_write(connection.socket, asio::buffer(this->robot_send_buffer, len + len_buffer.size()),
                              [this, &connection](const std::error_code& ec, std::size_t /*length*/) {
                                  if (ec) {
                                      LOG_WARNING(logger, "Could not robot command write. Error {}", ec.message());
                                      connection.connected = false;
                                  }
                              });
        };
    };

    if (color == TeamColor::BLUE) {
        do_work(this->blue_robot_connection);
    } else {
        do_work(this->yellow_robot_connection);
    }
}

void ErforceSimulationConnector::onSimulationCommand(const LuhsoccerSimulatorControl& control) {
    if (!this->simulation_connection.connected) return;
    uint32_t len = control.ByteSizeLong();
    auto len_buffer = lengthToBytes(len);

    // Fill the first 4 bytes of the buffer with the length
    std::copy(len_buffer.begin(), len_buffer.end(), this->simulation_send_buffer.begin());

    // Write the message into the buffer, with the offset of the length buffer
    if (control.SerializeToArray(this->simulation_send_buffer.data() + len_buffer.size(),
                                 static_cast<int>(this->simulation_send_buffer.size() - len_buffer.size()))) {
        asio::async_write(this->simulation_connection.socket,
                          asio::buffer(this->simulation_send_buffer, len + len_buffer.size()),
                          [this](const std::error_code& ec, std::size_t length) {
                              if (ec) {
                                  LOG_WARNING(logger, "Could not write control packet. Error {}", ec.message());
                                  this->simulation_connection.connected = false;
                              }
                          });
    }
}

void ErforceSimulationConnector::update() {
    if (config_provider::ConfigProvider::getConfigStore().game_config.is_blue &&
        this->current_controlling != TeamColor::BLUE) {
        closeRobotSocket(TeamColor::YELLOW);
        openRobotSocket(TeamColor::BLUE);
    } else if (!config_provider::ConfigProvider::getConfigStore().game_config.is_blue &&
               this->current_controlling != TeamColor::YELLOW) {
        closeRobotSocket(TeamColor::BLUE);
        openRobotSocket(TeamColor::YELLOW);
    }

    this->context.poll();
}

void ErforceSimulationConnector::stop() {
#ifdef _WIN32  // Only works on windows, @todo needs more investigation, same as in ssl interface
    std::error_code ec;
    simulation_connection.socket.shutdown(asio::socket_base::shutdown_both, ec);
    blue_robot_connection.socket.shutdown(asio::socket_base::shutdown_both, ec);
    blue_robot_connection.socket.shutdown(asio::socket_base::shutdown_both, ec);
    vision_connection.socket.shutdown(asio::socket_base::shutdown_both, ec);

    if (ec) {
        LOG_ERROR(logger, "Error while closing sockets: {} ({})", ec.message(), ec.value());
    }

#endif

    simulation_connection.socket.close();
    blue_robot_connection.socket.close();
    yellow_robot_connection.socket.close();
    vision_connection.socket.close();

    this->context.stop();
};

void ErforceSimulationConnector::printSimulationError(const SimulatorError& error) {
    if (error.has_message()) {
        LOG_WARNING(logger, "Got simulation control error message : {}", error.message());
    } else if (error.has_code()) {
        LOG_WARNING(logger, "Got simulation control error code: {}", error.code());
    } else {
        LOG_WARNING(logger, "Got simulation control error");
    }
}

}  // namespace luhsoccer::simulation
