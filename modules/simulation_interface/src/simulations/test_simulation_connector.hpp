#pragma once

#include "simulation_interface/simulation_connector.hpp"
#include "ssl_vision_wrapper.pb.h"
#include <cmath>

namespace luhsoccer::simulation {

class TestSimulationConnector : public SimulationConnector {
   public:
    TestSimulationConnector(VisionOutputCallback& vision_output, RobotOutputCallback& robot_feedback_output,
                            SimulationOutputCallback& simulation_feedback_output)
        : SimulationConnector(vision_output, robot_feedback_output, simulation_feedback_output) {}
    [[nodiscard]] SimulationConnectorType type() const override { return SimulationConnectorType::TEST_SIMULATION; };
    [[nodiscard]] time::Rate& getRate() override { return rate; };

    void update() override {
        proto::ssl_vision::SSL_WrapperPacket packet;
        auto detection = std::make_unique<proto::ssl_vision::SSL_DetectionFrame>();
        detection->set_t_capture(time::now().asSec());
        auto robot = detection->add_robots_blue();
        auto now = static_cast<float>(time::now().asSec());
        constexpr float RADIUS = 2000.0;
        robot->set_x(std::sin(now) * RADIUS);
        robot->set_y(std::cos(now) * RADIUS);
        robot->set_robot_id(1);
        constexpr float PI = 3.14;
        robot->set_orientation(std::sin(now) * PI);

        detection->set_t_sent(time::now().asSec());
        packet.set_allocated_detection(detection.release());
        this->vision_output(packet);
    }

   private:
    time::Rate rate{100.0, "TestSimulationConnector"};
};

}  // namespace luhsoccer::simulation