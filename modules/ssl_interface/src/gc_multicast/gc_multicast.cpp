#include "gc_multicast/gc_multicast.hpp"

#include <utility>
#include "logger/logger.hpp"
#include "ssl_types_converter/ssl_types_converter.hpp"
#include "ssl_interface/ssl_interface.hpp"

namespace luhsoccer::ssl_interface::connection {

GCMulticastConnection::GCMulticastConnection(SSLInterface& interface, event_system::EventSystem& event_system,
                                             const std::string& ip, uint16_t port)
    : interface(interface),
      port(port),
      event_system(event_system),
      socket(event_system.getIoContext()),
      listen_address(asio::ip::make_address_v4("0.0.0.0")),
      multicast_address(asio::ip::make_address_v4(ip)) {}

void GCMulticastConnection::setup() {
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

void GCMulticastConnection::read() {
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
            Referee packet;
            if (packet.ParseFromArray(this->receive_data.data(), static_cast<int>(length))) {
                this->interface.processGameControllerData<GameControllerDataSource::NETWORK>(
                    converter::parseRefereeData(packet));
            } else {
                logger.warning("Received malformed referee packet");
            }

            read();
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