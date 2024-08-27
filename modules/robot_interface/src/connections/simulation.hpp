#pragma once

#include "packets.hpp"
#include "robot_interface/robot_interface_types.hpp"
#include "luhsoccer_robot_interface.pb.h"

namespace luhsoccer::robot_interface {

class SimulationConnection {
    using RobotOutput = std::function<void(const proto::RobotFeedback&, TeamColor)>;

   public:
    SimulationConnection(std::function<void(RobotFeedbackWrapper)> on_feedback,
                         simulation_interface::SimulationInterface& interface);

    void send(const proto::RobotControl& cmd, TeamColor color);

   private:
    std::function<void(RobotFeedbackWrapper)> on_feedback;
    simulation_interface::SimulationInterface& interface;
};

class SimulationPacketBuilder : public PacketBuilder {
   public:
    SimulationPacketBuilder(SimulationConnection& connection);
    void addMessage(const RobotCommandWrapper& cmd) override;
    void buildAndSend() override;

   private:
    SimulationConnection& connection;

    bool send_blue;
    proto::RobotControl blue_control;

    bool send_yellow;
    proto::RobotControl yellow_control;
};

}  // namespace luhsoccer::robot_interface