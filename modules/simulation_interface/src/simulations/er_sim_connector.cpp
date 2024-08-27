#include "er_sim_connector.hpp"

#include "luhsoccer_simulation_control.pb.h"

#include "ssl_vision_wrapper.pb.h"

#include "ssl_simulation_robot_control.pb.h"
#include "ssl_simulation_robot_feedback.pb.h"
#include "ssl_simulation_control.pb.h"

namespace luhsoccer::simulation {

namespace {

::SimulatorCommand convertToSSLSimCommand(const LuhsoccerSimulatorControl& control) {
    ::SimulatorCommand ssl_control;

    if (control.has_control()) {
        ssl_control.mutable_control()->CopyFrom(control.control());
    }

    if (control.has_config()) {
        ssl_control.mutable_config()->CopyFrom(control.config());
    }

    return ssl_control;
}

::RobotControl convertToSSLCommand(const RobotControl& control) {
    ::RobotControl ssl_control;

    for (const auto& command : control.robot_commands()) {
        auto* ssl_command = ssl_control.add_robot_commands();

        ssl_command->set_id(command.id());

        if (command.has_move_command()) {
            auto& move_command = command.move_command();

            if (move_command.has_robot_velocity_control()) {
                auto* ssl_move_command = ssl_command->mutable_move_command();
                auto& velocity_control = move_command.robot_velocity_control();
                auto& ssl_local_velocity = *ssl_move_command->mutable_local_velocity();

                ssl_local_velocity.set_forward(velocity_control.desired_velocity().x());
                ssl_local_velocity.set_left(velocity_control.desired_velocity().y());
                ssl_local_velocity.set_angular(velocity_control.desired_velocity().z());
            } else {
                // TODO: Implement other move commands
            }
        }

        if (command.has_kick_command()) {
            auto& kick_command = command.kick_command();

            ssl_command->set_kick_speed(kick_command.kick_velocity());
        }

        if (command.has_kick_command()) {
            auto dribble_mode = command.dribbler_mode();

            switch (dribble_mode) {
                case proto::RobotCommand_DribblerMode_OFF:
                    ssl_command->set_dribbler_speed(0.0f);
                    break;
                case proto::RobotCommand_DribblerMode_LOW:
                    ssl_command->set_dribbler_speed(700.0f);
                    break;
                case proto::RobotCommand_DribblerMode_HIGH:
                    ssl_command->set_dribbler_speed(1400.0f);
                    break;
            }
        }
    }

    return ssl_control;
}

RobotControlResponse convertFromSSLCommand(const ::RobotControlResponse& ssl_response, bool is_blue) {
    RobotControlResponse response;

    for (const auto& ssl_feedback : ssl_response.feedback()) {
        auto* feedback = response.add_feedback();

        feedback->set_id(ssl_feedback.id());
        feedback->set_is_blue(is_blue);
        feedback->set_time_stamp(time::now().asNSec());
        feedback->set_robot_has_ball(ssl_feedback.dribbler_ball_contact());
    }

    return response;
}

}  // namespace

ErSimConnector::ErSimConnector(event_system::EventSystem& event_system, VisionOutputCallback& vision_output,
                               RobotOutputCallback& robot_feedback_output,
                               SimulationOutputCallback& simulation_feedback_output)
    : SimulationConnector(vision_output, robot_feedback_output, simulation_feedback_output),
      event_system(event_system),
      strand(event_system.getIoContext()),
      vision_socket(event_system.getIoContext()),
      robot_socket(event_system.getIoContext()),
      simulation_socket(event_system.getIoContext()) {}

void ErSimConnector::load() {
    uint16_t port = 26780;
    vision_socket.open(asio::ip::udp::v4());
    while (true) {
        asio::error_code ec;
        auto endpoint = asio::ip::udp::endpoint(asio::ip::make_address_v4("127.0.0.1"), port);
        vision_socket.bind(endpoint, ec);
        if (!ec) {
            // TODO add option to switch back to port - 1 if we don't receive packets after a certain time
            this->relay_endpoint = asio::ip::udp::endpoint(asio::ip::make_address_v4("127.0.0.1"), port + 1);
            break;
        } else if (ec == asio::error::address_in_use) {
            port++;
            logger.info("Port {} is already in use, trying port {}", port - 1, port);
        } else {
            logger.error("Error while binding vision socket: {}", ec.message());
            return;
        }
    }

    receiveVision();

    robot_socket.open(asio::ip::udp::v4());
    simulation_socket.open(asio::ip::udp::v4());
}

void ErSimConnector::receiveFeedback() {
    auto port = last_color == TeamColor::BLUE ? ROBOT_CONTROL_PORT_BLUE : ROBOT_CONTROL_PORT_YELLOW;
    asio::ip::udp::endpoint endpoint(asio::ip::make_address_v4("127.0.0.1"), port);
    robot_socket.async_receive_from(
        asio::buffer(this->robot_feedback_data), endpoint,
        asio::bind_executor(strand, [this](asio::error_code code, std::size_t length) {
            if (code) {
                logger.error("Error while receiving feedback data: {}", code.message());
                return;
            }
            ::RobotControlResponse feedback;

            if (feedback.ParseFromArray(this->robot_feedback_data.data(), static_cast<int>(length))) {
                for (const auto& feedback : feedback.errors()) {
                    logger.error("Received simulation error: {}", feedback.message());
                }

                RobotControlResponse response = convertFromSSLCommand(feedback, last_color == TeamColor::BLUE);
                for (const auto& feedback : response.feedback()) {
                    this->robot_feedback_output(feedback, last_color);
                }
            } else {
                logger.warning("Received malformed feedback packet. Read {} bytes", length);
            }
        }));
}

void ErSimConnector::onSimulationCommand(const LuhsoccerSimulatorControl& control) {
    ::SimulatorCommand ssl_control = convertToSSLSimCommand(control);

    uint32_t len = ssl_control.ByteSizeLong();

    if (ssl_control.SerializeToArray(this->simulator_control_data.data(),
                                     static_cast<int>(this->simulator_control_data.size()))) {
        simulation_socket.async_send_to(asio::buffer(this->simulator_control_data, len),
                                        asio::ip::udp::endpoint(asio::ip::make_address_v4("127.0.0.1"), 10300),
                                        asio::bind_executor(strand, [](asio::error_code code, std::size_t length) {}));
    }
}

void ErSimConnector::receiveVision() {
    vision_socket.async_receive(
        asio::buffer(this->vision_data), asio::bind_executor(strand, [this](asio::error_code code, std::size_t length) {
            if (code) {
                logger.error("Error while receiving vision data: {}", code.message());
                return;
            }

            ssl_vision::SSL_WrapperPacket wrapper;
            if (wrapper.ParseFromArray(this->vision_data.data(), static_cast<int>(length))) {
                this->vision_output(wrapper);
            } else {
                logger.warning("Received malformed vision packet. Read {} bytes", length);
            }
            vision_socket.async_send_to(asio::buffer(this->vision_data, length), this->relay_endpoint,
                                        asio::bind_executor(strand, [this](asio::error_code code, std::size_t length) {
                                            this->receiveVision();
                                        }));
        }));
}

void ErSimConnector::onRobotCommand(const RobotControl& control, const TeamColor color) {
    ::RobotControl ssl_control = convertToSSLCommand(control);
    uint32_t len = ssl_control.ByteSizeLong();

    last_color = color;

    static asio::ip::address_v4 address = asio::ip::make_address_v4("127.0.0.1");
    auto port = color == TeamColor::BLUE ? ROBOT_CONTROL_PORT_BLUE : ROBOT_CONTROL_PORT_YELLOW;

    // Write the message into the buffer, with the offset of the length buffer
    if (ssl_control.SerializeToArray(this->robot_send_data.data(), static_cast<int>(this->robot_send_data.size()))) {
        robot_socket.async_send_to(asio::buffer(this->robot_send_data.data(), len),
                                   asio::ip::udp::endpoint(address, port),
                                   asio::bind_executor(strand, [this](asio::error_code code, std::size_t /*length*/) {
                                       if (code) {
                                           fmt::print("Error while sending robot data: {}", code.message());
                                           return;
                                       }

                                       this->receiveFeedback();
                                   }));
    };
}

void ErSimConnector::stop() {
#ifdef _WIN32
    std::error_code ec;
    vision_socket.shutdown(vision_socket.shutdown_both, ec);
    robot_socket.shutdown(robot_socket.shutdown_both, ec);
    simulation_socket.shutdown(simulation_socket.shutdown_both, ec);
#endif

    vision_socket.close();
    robot_socket.close();
    simulation_socket.close();
}

}  // namespace luhsoccer::simulation
