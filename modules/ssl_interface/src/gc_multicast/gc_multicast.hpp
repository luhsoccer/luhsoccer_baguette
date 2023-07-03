#pragma once

#include "ssl_interface/ssl_interface.hpp"
#include <array>
#include <asio.hpp>
#include <asio/buffer.hpp>
#include <asio/io_context.hpp>
#include <asio/ip/udp.hpp>
#include "logger/logger.hpp"

namespace luhsoccer::ssl_interface::connection {

constexpr int SSL_GC_MULTICAST_DEFAULT_PORT = 10003;

class GCMulticastConnection {
   public:
    GCMulticastConnection(SSLInterface& callback, asio::io_context& context, const std::string& ip = "224.5.23.1",
                          uint16_t port = SSL_GC_MULTICAST_DEFAULT_PORT);
    void setup();
    void read();
    void close();

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
    logger::Logger logger{"gc_multicast_connection"};
};
}  // namespace luhsoccer::ssl_interface::connection