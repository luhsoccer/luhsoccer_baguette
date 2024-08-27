#pragma once

#include "event_system/event_system.hpp"
#include "simulation_interface/simulation_connector.hpp"

#include "asio.hpp"

#include "luhsoccer_robot_interface.pb.h"

namespace luhsoccer::simulation {

class ErSimConnector : public SimulationConnector {
   public:
    ErSimConnector(event_system::EventSystem& event_system, VisionOutputCallback& vision_output,
                   RobotOutputCallback& robot_feedback_output, SimulationOutputCallback& simulation_feedback_output);

    [[nodiscard]] SimulationConnectorType type() const override { return SimulationConnectorType::ER_SIM; };

    void load() override;

    void stop() override;

    void onRobotCommand(const RobotControl& control, const TeamColor color) override;

    void onSimulationCommand(const LuhsoccerSimulatorControl& control) override;

   private:
    void receiveVision();
    void receiveFeedback();

    event_system::EventSystem& event_system;

    constexpr static unsigned int ROBOT_CONTROL_PORT_BLUE = 10301;
    constexpr static unsigned int ROBOT_CONTROL_PORT_YELLOW = 10302;

    std::atomic<TeamColor> last_color{TeamColor::BLUE};

    constexpr static unsigned int BUFFER_SIZE = 8192;  // 8 kb buffer size by default

    std::array<char, BUFFER_SIZE> vision_data{};

    std::array<char, BUFFER_SIZE> robot_send_data{};

    std::array<char, BUFFER_SIZE> robot_feedback_data{};

    std::array<char, BUFFER_SIZE> simulator_control_data{};

    logger::Logger logger{"ErSimConnector"};

    asio::ip::udp::socket vision_socket;
    asio::ip::udp::socket robot_socket;
    asio::ip::udp::socket simulation_socket;
    asio::ip::udp::endpoint relay_endpoint;
    asio::io_context::strand strand;
};

}  // namespace luhsoccer::simulation