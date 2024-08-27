#pragma once

#include "ssl_interface/ssl_interface.hpp"
#include "logger/logger.hpp"
#include <asio.hpp>
#include <array>

namespace luhsoccer::ssl_interface::connection {

constexpr uint16_t SSL_GC_TCP_DEFAULT_PORT = 10009;

class GCTcpConnection {
   public:
    GCTcpConnection(SSLInterface& callback, event_system::EventSystem& event_system,
                    const std::string& ip = "127.0.0.1", uint16_t port = SSL_GC_TCP_DEFAULT_PORT);

    void setup();

    void writePacket();
    void readPacket();
    void close();

    void connect();

   private:
    SSLInterface& interface;
    event_system::EventSystem& event_system;
    uint16_t port;

    asio::ip::tcp::endpoint endpoint;
    asio::steady_timer reconnect_timer;

    constexpr static unsigned int BUFFER_SIZE = 8192;
    std::array<char, BUFFER_SIZE> send_data{};
    std::array<char, BUFFER_SIZE> receive_data{};
    asio::mutable_buffer receive_buffer{receive_data.data(), receive_data.size()};
    asio::ip::tcp::socket socket;

    asio::ip::address_v4 gc_address;
    logger::Logger logger{"gc_tcp_connection"};
};

}  // namespace luhsoccer::ssl_interface::connection