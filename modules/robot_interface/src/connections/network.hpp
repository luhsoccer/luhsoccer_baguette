#pragma once

#include "luhsoccer_robot_interface.pb.h"
#include "luhsoccer_basestation.pb.h"
#include "logger/logger.hpp"
#include "packets.hpp"
#include <asio.hpp>

namespace luhsoccer::robot_interface {

class NetworkConnection {
    using RobotOutput = std::function<void(const proto::RobotFeedback&)>;

   public:
    void setup();
    void updateBSHost();
    std::optional<proto::basestation::FromBasestationWrapper> sendPacket(
        const proto::basestation::ToBasestationWrapper& control);
    void stop();

   private:
    std::optional<asio::ip::udp::endpoint> endpoint;

    unsigned int last_sequence_number = 0;
    logger::Logger logger{"NetworkConnection"};
    asio::io_context context;
    asio::ip::udp::socket socket{context};
    asio::ip::udp::resolver resolver{context};
};

class NetworkPacketBuilder : public PacketBuilder {
   public:
    NetworkPacketBuilder(NetworkConnection& connection);
    void addMessage(const RobotCommandWrapper& cmd) override;
    std::vector<RobotFeedbackWrapper> buildAndSend() override;

   private:
    NetworkConnection& connection;

    proto::basestation::ToBasestationWrapper wrapper;
};

}  // namespace luhsoccer::robot_interface