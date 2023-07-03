#pragma once

#include "ssl_interface/ssl_interface.hpp"
#include <array>
#include <asio.hpp>
#include <asio/buffer.hpp>
#include <asio/io_context.hpp>
#include <asio/ip/udp.hpp>
#include "logger/logger.hpp"

namespace luhsoccer::ssl_interface::connection {

constexpr int SSL_VISION_MULTICAST_DEFAULT_PORT = 10006;

class VisionMulticastConnection {
   public:
    VisionMulticastConnection(SSLInterface& callback, asio::io_context& context, const std::string& ip = "224.5.23.2",
                              uint16_t port = SSL_VISION_MULTICAST_DEFAULT_PORT);
    void setup();
    void read();
    void close();
    void publish(const SSLWrapperData& data);

   private:
    SSLInterface& interface;

    asio::io_context& context;

    uint16_t port;

    constexpr static unsigned int BUFFER_SIZE = 8192;  // 8 kb buffer size by default
    std::array<char, BUFFER_SIZE> receive_data{};
    asio::mutable_buffer receive_buffer{receive_data.data(), receive_data.size()};
    asio::ip::udp::socket socket;

    asio::ip::address listen_address;
    asio::ip::address multicast_address;
    logger::Logger logger{"vision_multicast_connection"};
};
}  // namespace luhsoccer::ssl_interface::connection