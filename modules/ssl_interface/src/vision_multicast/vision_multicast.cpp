#include "vision_multicast/vision_multicast.hpp"

#include "config_provider/config_store_main.hpp"

#include <utility>
#include "logger/logger.hpp"
#include "ssl_types_converter/ssl_types_converter.hpp"
#include "ssl_interface/ssl_interface.hpp"

#include "ssl_vision_wrapper.pb.h"
namespace luhsoccer::ssl_interface::connection {

VisionMulticastConnection::VisionMulticastConnection(SSLInterface& interface, asio::io_context& context,
                                                     const std::string& ip, uint16_t port)
    : interface(interface),
      context(context),
      port(port),
      socket(context),
      listen_address(asio::ip::make_address_v4("0.0.0.0")),
      multicast_address(asio::ip::make_address_v4(ip)) {}

void VisionMulticastConnection::setup() {
    using namespace asio::ip;
    // TODO get ip and port via param server
    auto listen_address = make_address_v4("0.0.0.0");
    LOG_DEBUG(logger, "Setup multicast connection on group {}:{}", multicast_address.to_string(), port);

    const udp::endpoint endpoint(listen_address, port);

    try {
        // Bind the socket to the local interface
        this->socket.open(endpoint.protocol());
        this->socket.set_option(udp::socket::reuse_address(true));
        this->socket.bind(endpoint);

        // Join the multicast group
        this->socket.set_option(multicast::join_group(multicast_address));
    } catch (asio::system_error& e) {
        LOG_ERROR(logger, "{}", e.what());
    }
}

void VisionMulticastConnection::read() {
    static const auto& ssl_config = config_provider::ConfigProvider::getConfigStore().ssl_interface_config;

    this->socket.async_receive(this->receive_buffer, [this](asio::error_code code, std::size_t length) {
        if (code) {
            LOG_WARNING(logger, "Got error code {} with message: {}", code.value(), code.message());
        } else {
            proto::ssl_vision::SSL_WrapperPacket packet;
            if (packet.ParseFromArray(this->receive_data.data(), static_cast<int>(length))) {
                if (packet.detection().camera_id() != ssl_config.ignore_camera) {
                    this->interface.processWrapperData<VisionDataSource::NETWORK>(converter::parseWrapperData(packet));
                }
            } else {
                LOG_WARNING(logger, "Received malformed ssl wrapper packet");
            }

            if (!this->context.stopped()) {
                read();
            }
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
    static asio::ip::udp::endpoint send_endpoint(multicast_address, port);

    std::string buffer = converter::serializeWrapperData(data).SerializeAsString();
    asio::const_buffer network_buffer(buffer.c_str(), buffer.length());
    this->socket.send_to(network_buffer, send_endpoint);
}

}  // namespace luhsoccer::ssl_interface::connection