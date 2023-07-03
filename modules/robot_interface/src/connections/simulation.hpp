#include "packets.hpp"
#include "robot_interface/robot_interface_types.hpp"
#include "luhsoccer_robot_interface.pb.h"

namespace luhsoccer::robot_interface {

class SimulationConnection {
    using RobotOutput = std::function<void(const proto::RobotFeedback&, TeamColor)>;

   public:
    SimulationConnection(simulation_interface::SimulationInterface& interface);
    void setRobotOutput(RobotOutput output);
    void send(const proto::RobotControl& cmd, TeamColor color);

    std::mutex last_feedbacks_mutex;
    std::vector<proto::RobotFeedback> last_feedbacks;

   private:
    simulation_interface::SimulationInterface& interface;
};

class SimulationPacketBuilder : public PacketBuilder {
   public:
    SimulationPacketBuilder(SimulationConnection& connection);
    void addMessage(const RobotCommandWrapper& cmd) override;
    std::vector<RobotFeedbackWrapper> buildAndSend() override;

   private:
    SimulationConnection& connection;

    bool send_blue;
    proto::RobotControl blue_control;

    bool send_yellow;
    proto::RobotControl yellow_control;
};

}  // namespace luhsoccer::robot_interface