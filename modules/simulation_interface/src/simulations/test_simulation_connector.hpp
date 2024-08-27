#pragma once

#include "simulation_interface/simulation_connector.hpp"
#include "ssl_vision_wrapper.pb.h"
#include <cmath>

#include "event_system/event_system.hpp"
#include "event_system/timer_events.hpp"

namespace luhsoccer::simulation {

class TestSimulationConnector : public SimulationConnector {
   public:
    TestSimulationConnector(event_system::EventSystem& event_system, VisionOutputCallback& vision_output,
                            RobotOutputCallback& robot_feedback_output,
                            SimulationOutputCallback& simulation_feedback_output)
        : SimulationConnector(vision_output, robot_feedback_output, simulation_feedback_output) {
        event_system.registerEventHandler<event_system::TimerEvent100Hz>(
            [&](event_system::EventContext<event_system::TimerEvent100Hz> /*ctx*/) { this->update(); });
    }
    [[nodiscard]] SimulationConnectorType type() const override { return SimulationConnectorType::TEST_SIMULATION; };

    void load() override { running = true; }

    void stop() override { running = false; }

    void update() {
        if (!running) {
            return;
        }

        proto::ssl_vision::SSL_WrapperPacket packet;
        auto detection = packet.mutable_detection();
        detection->set_t_capture(time::now().asSec());
        auto blue_robot = detection->add_robots_blue();
        auto now = static_cast<float>(time::now().asSec());
        constexpr float RADIUS = 2000.0;
        blue_robot->set_x(std::sin(now) * RADIUS);
        blue_robot->set_y(std::cos(now) * RADIUS);
        blue_robot->set_robot_id(1);
        constexpr float PI = 3.14;
        blue_robot->set_orientation(std::sin(now) * PI);

        auto yellow_robot = detection->add_robots_yellow();
        yellow_robot->set_x(std::sin(now) * -RADIUS);
        yellow_robot->set_y(std::cos(now) * -RADIUS);
        yellow_robot->set_robot_id(2);
        yellow_robot->set_orientation(-std::sin(now) * PI);

        detection->set_t_sent(time::now().asSec());

        auto center = packet.mutable_geometry()->mutable_field()->mutable_field_arcs()->Add();
        center->set_name("CenterCircle");
        center->set_radius(RADIUS);
        center->set_a1(0.0);
        center->set_thickness(50);
        center->set_a2(2 * PI);
        auto center_pos = center->mutable_center();
        center_pos->set_x(0.0);
        center_pos->set_y(0.0);

        this->vision_output(packet);
    }

   private:
    std::atomic_bool running{false};
};

}  // namespace luhsoccer::simulation