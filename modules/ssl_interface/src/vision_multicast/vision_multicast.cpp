#include "vision_multicast/vision_multicast.hpp"

#include "config_provider/config_store_main.hpp"
#include "config/ssl_interface_config.hpp"

#include <utility>
#include "logger/logger.hpp"
#include "ssl_types_converter/ssl_types_converter.hpp"
#include "ssl_interface/ssl_interface.hpp"

#include "ssl_vision_wrapper.pb.h"
namespace luhsoccer::ssl_interface::connection {

namespace {

void filterVisionData(SSLVisionData& data, const int side_to_ignore) {
    if (side_to_ignore != 0) {
        auto balls_it = data.balls.begin();
        while (balls_it != data.balls.end()) {
            if (side_to_ignore > 0) {
                if (balls_it->position.x() > 0) {
                    balls_it = data.balls.erase(balls_it);
                } else {
                    balls_it++;
                }
            } else {
                if (balls_it->position.x() < 0) {
                    balls_it = data.balls.erase(balls_it);
                } else {
                    balls_it++;
                }
            }
        }

        auto blue_robot_it = data.blue_robots.begin();
        while (blue_robot_it != data.blue_robots.end()) {
            if (side_to_ignore > 0) {
                if (blue_robot_it->transform.translation().x() > 0) {
                    blue_robot_it = data.blue_robots.erase(blue_robot_it);
                } else {
                    blue_robot_it++;
                }
            } else {
                if (blue_robot_it->transform.translation().x() < 0) {
                    blue_robot_it = data.blue_robots.erase(blue_robot_it);
                } else {
                    blue_robot_it++;
                }
            }
        }

        auto yellow_robot_it = data.yellow_robots.begin();
        while (yellow_robot_it != data.yellow_robots.end()) {
            if (side_to_ignore > 0) {
                if (yellow_robot_it->transform.translation().x() > 0) {
                    yellow_robot_it = data.yellow_robots.erase(yellow_robot_it);
                } else {
                    yellow_robot_it++;
                }
            } else {
                if (yellow_robot_it->transform.translation().x() < 0) {
                    yellow_robot_it = data.yellow_robots.erase(yellow_robot_it);
                } else {
                    yellow_robot_it++;
                }
            }
        }
    }
}
}  // namespace

VisionMulticastConnection::VisionMulticastConnection(SSLInterface& interface, event_system::EventSystem& event_system,
                                                     const std::string& ip, uint16_t port)
    : interface(interface),
      event_system(event_system),
      port(port),
      socket(event_system.getIoContext()),
      listen_address(asio::ip::make_address_v4("0.0.0.0")),
      multicast_address(asio::ip::make_address_v4(ip)) {}

void VisionMulticastConnection::setup() {
    using namespace asio::ip;
    // TODO get ip and port via param server
    auto listen_address = make_address_v4("0.0.0.0");
    logger.debug("Setup multicast connection on group {}:{}", multicast_address.to_string(), port);

    const udp::endpoint endpoint(listen_address, port);

    try {
        // Bind the socket to the local interface
        this->socket.open(endpoint.protocol());
        this->socket.set_option(udp::socket::reuse_address(true));
        this->socket.bind(endpoint);

        // Join the multicast group
        this->socket.set_option(multicast::join_group(multicast_address));
        this->last_multicast_set = time::now();
    } catch (asio::system_error& e) {
        logger.error("{}", e.what());
    }
}

void VisionMulticastConnection::read() {
    static const auto& ssl_config = config_provider::ConfigProvider::getConfigStore().ssl_interface_config;

    this->socket.async_receive(this->receive_buffer, [this](asio::error_code code, std::size_t length) {
        if (code) {
            logger.warning("Got error code {} with message: {}", code.value(), code.message());
        } else {
            auto diff = time::now() - this->last_multicast_set;
            if (diff > std::chrono::seconds(10)) {
                logger.info("Join multicast group again");
                this->socket.set_option(asio::ip::multicast::leave_group(multicast_address));
                this->socket.set_option(asio::ip::multicast::join_group(multicast_address));
                this->last_multicast_set = time::now();
            }
            proto::ssl_vision::SSL_WrapperPacket packet;
            if (packet.ParseFromArray(this->receive_data.data(), static_cast<int>(length))) {
                if (packet.detection().camera_id() != ssl_config.ignore_camera) {
                    luhsoccer::ssl_interface::SSLWrapperData data = converter::parseWrapperData(packet);

                    // Look which data we should ignore (positive or negative halve (if specified in config))
                    if (data.vision.has_value()) {
                        const int side_to_ignore = ssl_config.ignore_side;
                        filterVisionData(*data.vision, side_to_ignore);
                    }

                    this->interface.processWrapperData<VisionDataSource::NETWORK>(data);
                }
            } else {
                logger.warning("Received malformed ssl wrapper packet");
            }

            read();
        }
    });
}

void VisionMulticastConnection::close() {
#ifdef _WIN32  // Only works on windows, @todo needs more investigation
    this->socket.shutdown(asio::socket_base::shutdown_both);
#endif
    this->socket.close();
}

void VisionMulticastConnection::publish(const SSLWrapperData& data) {
    // TODO refactor
    /*static asio::ip::udp::endpoint send_endpoint(multicast_address, port);

    std::string buffer = converter::serializeWrapperData(data).SerializeAsString();
    asio::const_buffer network_buffer(buffer.c_str(), buffer.length());
    this->socket.send_to(network_buffer, send_endpoint);*/
}

}  // namespace luhsoccer::ssl_interface::connection