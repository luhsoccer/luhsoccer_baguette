#include "ssl_interface/ssl_interface.hpp"
#include "logger/logger.hpp"
#include <asio.hpp>
#include <array>

namespace luhsoccer::ssl_interface::connection {

constexpr uint16_t SSL_GC_TCP_DEFAULT_PORT = 10009;

class GCTcpConnection {
   public:
    GCTcpConnection(SSLInterface& callback, asio::io_context& context, const std::string& ip = "127.0.0.1",
                    uint16_t port = SSL_GC_TCP_DEFAULT_PORT);

    void setup();
    void read();
    void close();

   private:
    SSLInterface& interface;
    asio::io_context& context;
    uint16_t port;

    constexpr static unsigned int BUFFER_SIZE = 8192;
    std::array<char, BUFFER_SIZE> receive_data{};
    asio::mutable_buffer receive_buffer{receive_data.data(), receive_data.size()};
    asio::ip::tcp::socket socket;

    asio::ip::address gc_address;
    logger::Logger logger{"gc_tcp_connection"};
};

}  // namespace luhsoccer::ssl_interface::connection