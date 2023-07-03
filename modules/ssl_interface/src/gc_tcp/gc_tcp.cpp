
#include "gc_tcp/gc_tcp.hpp"
#include <bitset>
#include "ssl_gc_ci.pb.h"
#include "ssl_types_converter/ssl_types_converter.hpp"

namespace luhsoccer::ssl_interface::connection {

namespace {
/**
 * @brief Decodes a uvariant encoded protobuf message size prefix
 *
 * @param receive_buffer The Buffer containing the received Message
 * @param received_size Size of the Message that was received
 * @return std::optional<std::pair<uint32_t, std::size_t>> A pair containing the computed size and the number of bytes
 * used for the size
 *
 * See https://github.com/RoboCup-SSL/ssl-game-controller/tree/master/cmd/ssl-ci-test-client
 * and https://cwiki.apache.org/confluence/display/GEODE/Delimiting+Protobuf+Messages
 *
 */
std::optional<std::pair<uint32_t, std::size_t>> decodeUvariant(const asio::mutable_buffer& receive_buffer,
                                                               std::size_t received_size) {
    const auto mes_bytes = static_cast<uint8_t*>(receive_buffer.data());

    // In uvariant encoding the MSB is reserved to indicate that another bit is needed for transmitting the
    // size. Thats why it cant be used for calculating the size
    constexpr uint8_t USABLE_UVARIANT_BITS = 7;
    constexpr uint8_t MSB_MASK = 0x80;           // 0b10000000
    constexpr uint8_t NEG_MSB_MASK = ~MSB_MASK;  // 0b01111111

    std::size_t index = 0;
    uint32_t computed_size = 0;
    uint8_t byte = 0;

    if (received_size == 0) return std::nullopt;

    do {
        if (index >= received_size - 1) return std::nullopt;  // failsafe
        byte = mes_bytes[index];
        computed_size |= ((byte & NEG_MSB_MASK) << USABLE_UVARIANT_BITS * index);
    } while (++index, (byte & MSB_MASK) == MSB_MASK);

    return std::pair{computed_size, index};
}
}  // namespace

GCTcpConnection::GCTcpConnection(SSLInterface& callback, asio::io_context& context, const std::string& ip,
                                 uint16_t port)
    : interface(callback), context(context), socket(context), gc_address(asio::ip::make_address_v4(ip)), port(port) {}

void GCTcpConnection::setup() {
    auto endpoint = asio::ip::tcp::endpoint(this->gc_address, this->port);
    try {
        this->socket.connect(endpoint);

    } catch (asio::system_error& e) {
        LOG_WARNING(this->logger, "Could not connect to the Game Controller via TCP! ({})", e.what());
    }

    if (this->socket.is_open()) {
        LOG_INFO(this->logger, "Connected to Game Controller via TCP");
    }
}

void GCTcpConnection::read() {
    this->socket.async_receive(this->receive_buffer, [this](asio::error_code code, std::size_t length) {
        if (code) {
            LOG_WARNING(this->logger, "Got error code {} with message: {}", code.value(), code.message());
        } else {
            bool packet_valid = false;

            const auto packet_size_data = decodeUvariant(this->receive_buffer, length);

            if (packet_size_data.has_value()) {
                const auto [packet_size, size_bits] = *packet_size_data;

                CiOutput packet;
                if (packet.ParseFromArray(this->receive_data.data() + size_bits, static_cast<int>(packet_size))) {
                    packet_valid = true;
                    if (packet.has_referee_msg()) {
                        const auto ref_packet = converter::parseRefereeData(packet.referee_msg());
                        this->interface.processGameControllerData<GameControllerDataSource::INTERNAL>(ref_packet);
                    }
                }
            }

            if (!packet_valid) {
                LOG_WARNING(logger, "Received malformed GC-CI packet");
            }

            if (!this->context.stopped()) {
                read();
            }
        }
    });
}

void GCTcpConnection::close() {
    try {
#ifdef _WIN32  // Only works on windows, @todo needs more investigation
        this->socket.shutdown(asio::socket_base::shutdown_both);
#endif
        this->socket.close();
    } catch (asio::system_error& e) {
        LOG_WARNING(this->logger, "Could not correctly close Socket! ({})", e.what());
    }
}

}  // namespace luhsoccer::ssl_interface::connection