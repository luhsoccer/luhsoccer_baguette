#include "gc_multicast/gc_multicast.hpp"

#include <utility>
#include "logger/logger.hpp"
#include "ssl_types_converter/ssl_types_converter.hpp"
#include "ssl_interface/ssl_interface.hpp"

#include "ssl_vision_wrapper.pb.h"
namespace luhsoccer::ssl_interface::connection {

GCMulticastConnection::GCMulticastConnection(SSLInterface& interface, asio::io_context& context, const std::string& ip,
                                             uint16_t port)
    : interface(interface),
      context(context),
      port(port),
      socket(context),
      listen_address(asio::ip::make_address_v4("0.0.0.0")),
      multicast_address(asio::ip::make_address_v4(ip)) {}

void GCMulticastConnection::setup() {
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

void GCMulticastConnection::read() {
    this->socket.async_receive(this->receive_buffer, [this](asio::error_code code, std::size_t length) {
        if (code) {
            LOG_WARNING(logger, "Got error code {} with message: {}", code.value(), code.message());
        } else {
            Referee packet;
            if (packet.ParseFromArray(this->receive_data.data(), static_cast<int>(length))) {
                this->interface.processGameControllerData<GameControllerDataSource::NETWORK>(
                    converter::parseRefereeData(packet));
            } else {
                LOG_WARNING(logger, "Received malformed referee packet");
            }

            if (!this->context.stopped()) {
                read();
            }
        }
    });
}

void GCMulticastConnection::close() {
#ifdef _WIN32  // Only works on windows, @todo needs more investigation
    this->socket.shutdown(asio::socket_base::shutdown_both);
#endif
    this->socket.close();
}

}  // namespace luhsoccer::ssl_interface::connection