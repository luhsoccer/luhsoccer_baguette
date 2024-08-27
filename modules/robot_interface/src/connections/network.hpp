#pragma once

#include "luhsoccer_basestation.pb.h"
#include "logger/logger.hpp"
#include "packets.hpp"
#include <asio.hpp>
#include <bitset>

namespace luhsoccer::robot_interface {

class NetworkConnection {
   public:
    NetworkConnection(std::function<void(RobotFeedbackWrapper)> on_feedback, asio::io_context& ctx)
        : on_feedback(std::move(on_feedback)), ctx(ctx), socket(ctx), resolver(ctx) {
        this->setup();
    }

    void setup();
    void updateBSHost();
    void sendPacket(const proto::basestation::ToBasestationWrapper& bs1_control,
                    const proto::basestation::ToBasestationWrapper& bs2_control);
    void stop();

   private:
    bool resolveHostname(asio::ip::udp::endpoint& target, const std::string& hostname, const uint32_t port,
                         bool takefirst = false);

    void receiveData();

   public:
    std::bitset<16> bs1_robots;

   private:
    std::optional<asio::ip::udp::endpoint> endpoint;
    std::optional<asio::ip::udp::endpoint> second_endpoint;

    // use asio strands
    std::mutex mutex;
    std::unordered_map<asio::ip::address_v4::uint_type, uint32_t> sequence_ids;
    asio::ip::udp::endpoint received_endpoint;
    std::array<uint8_t, 4096> receive_buffer;

    std::function<void(RobotFeedbackWrapper)> on_feedback;

    unsigned int last_sequence_number = 0;
    logger::Logger logger{"NetworkConnection"};
    asio::io_context& ctx;
    asio::ip::udp::socket socket;
    asio::ip::udp::resolver resolver;
};

class NetworkPacketBuilder : public PacketBuilder {
   public:
    NetworkPacketBuilder(NetworkConnection& connection);
    void addMessage(const RobotCommandWrapper& cmd) override;
    void buildAndSend() override;

   private:
    NetworkConnection& connection;

    proto::basestation::ToBasestationWrapper wrapper;
    proto::basestation::ToBasestationWrapper second_bs_wrapper;
};

}  // namespace luhsoccer::robot_interface