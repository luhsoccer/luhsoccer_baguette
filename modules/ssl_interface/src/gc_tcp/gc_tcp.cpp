
#include "gc_tcp/gc_tcp.hpp"
#include <bitset>
#include "ssl_gc_ci.pb.h"
#include "ssl_types_converter/ssl_types_converter.hpp"
#include "google/protobuf/io/coded_stream.h"

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

GCTcpConnection::GCTcpConnection(SSLInterface& callback, event_system::EventSystem& event_system, const std::string& ip,
                                 uint16_t port)
    : interface(callback),
      event_system(event_system),
      socket(event_system.getIoContext()),
      gc_address(asio::ip::make_address_v4(ip)),
      port(port),
      reconnect_timer(event_system.getIoContext()),
      endpoint(this->gc_address, this->port) {}

void GCTcpConnection::setup() { this->connect(); }

void GCTcpConnection::connect() {
    this->socket.async_connect(this->endpoint, [this](asio::error_code code) {
        if (code) {
            // this->logger.warning("Could not connect to Game Controller via TCP: {}", code.message());
            this->reconnect_timer.expires_after(std::chrono::seconds(3));
            this->reconnect_timer.async_wait([this](asio::error_code code) {
                if (!code) {
                    this->connect();
                }
            });
        } else {
            this->logger.info("Connected to Game Controller via TCP");
            this->writePacket();
        }
    });
}

void GCTcpConnection::writePacket() {
    CiInput packet;
    packet.set_timestamp(std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::high_resolution_clock::now().time_since_epoch())
                             .count());

    int packet_size = packet.ByteSize();

    google::protobuf::io::ArrayOutputStream output_stream(this->send_data.data(),
                                                          static_cast<int>(this->send_data.size()));
    google::protobuf::io::CodedOutputStream coded_output(&output_stream);

    coded_output.WriteVarint32(packet.ByteSizeLong());
    packet.SerializeToCodedStream(&coded_output);

    asio::mutable_buffer send_buffer{send_data.data(),
                                     packet_size + google::protobuf::io::CodedOutputStream::VarintSize32(packet_size)};

    this->socket.async_send(send_buffer, [this](asio::error_code code, std::size_t /*length*/) {
        if (code) {
            this->logger.warning("Could not send packet to Game Controller via TCP: {}. Try to reconnect!",
                                 code.message());
            this->reconnect_timer.expires_after(std::chrono::seconds(3));
            this->reconnect_timer.async_wait([this](asio::error_code code) {
                if (!code) {
                    this->connect();
                }
            });
        } else {
            readPacket();
        }
    });
}

void GCTcpConnection::readPacket() {
    this->socket.async_receive(this->receive_buffer, [this](asio::error_code code, std::size_t length) {
        if (code) {
            this->logger.warning("Could not read packet from Game Controller via TCP: {}. Try to reconnect!",
                                 code.message());
            this->reconnect_timer.expires_after(std::chrono::seconds(3));
            this->reconnect_timer.async_wait([this](asio::error_code code) {
                if (!code) {
                    this->connect();
                }
            });
        } else {
            bool packet_valid = false;

            google::protobuf::io::ArrayInputStream input_stream(this->receive_data.data(), static_cast<int>(length));
            google::protobuf::io::CodedInputStream coded_input(&input_stream);

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
                logger.warning("Received malformed GC-CI packet");
            }

            this->reconnect_timer.expires_after(std::chrono::milliseconds(10));
            this->reconnect_timer.async_wait([this](asio::error_code code) {
                if (!code) {
                    writePacket();
                }
            });
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
        this->logger.warning("Could not correctly close Socket! ({})", e.what());
    }
}

}  // namespace luhsoccer::ssl_interface::connection