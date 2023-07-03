#pragma once

#include <utility>

#include "simulation_interface/simulation_connector.hpp"
#include "logger/logger.hpp"
#include "asio.hpp"

class SimulatorError;

namespace luhsoccer::simulation {
class ErforceSimulationConnector : public SimulationConnector {
   public:
    ErforceSimulationConnector(VisionOutputCallback& vision_output, RobotOutputCallback& robot_feedback_output,
                               SimulationOutputCallback& simulation_feedback_output)
        : SimulationConnector(vision_output, robot_feedback_output, simulation_feedback_output) {}

    [[nodiscard]] SimulationConnectorType type() const override { return SimulationConnectorType::ERFORCE_SIMULATION; };
    [[nodiscard]] time::Rate& getRate() override { return this->rate; };

    void load() override;

    void update() override;

    void stop() override;

   private:
    constexpr static std::size_t BUFFER_SIZE = 8192;  // 8 kb buffer size by default
    constexpr static std::size_t HEADER_BUFFER_SIZE = sizeof(uint32_t);

    using HeaderBuffer = std::array<std::byte, HEADER_BUFFER_SIZE>;
    using DataBuffer = std::array<std::byte, BUFFER_SIZE>;

    struct SocketConnection {
        SocketConnection(std::string host, std::string port, asio::io_context& context,
                         void (ErforceSimulationConnector::*callback)(SocketConnection&, uint32_t))
            : host(std::move(host)),
              port(std::move(port)),
              socket(context),
              reconnect_timer(context),
              callback(callback) {}
        std::string host;
        std::string port;
        asio::ip::tcp::socket socket;
        asio::steady_timer reconnect_timer;
        void (ErforceSimulationConnector::*callback)(SocketConnection&, uint32_t);
        std::atomic_bool connected{false};
        std::atomic_bool connecting{false};
        HeaderBuffer header_buffer{};
        DataBuffer data_buffer{};
    };

    std::array<std::byte, BUFFER_SIZE> robot_send_buffer{};
    std::array<std::byte, BUFFER_SIZE> simulation_send_buffer{};

    template <typename T>
    void handleSimulationError(T message) {
        for (const SimulatorError& error : message.errors()) {
            printSimulationError(error);
        }
    }

    static inline constexpr int BYTE_MASK = 0b11111111;
    static inline constexpr int FIRST_BYTE = 0;
    static inline constexpr int SECOND_BYTE = 8;
    static inline constexpr int THIRD_BYTE = 16;
    static inline constexpr int FORTH_BYTE = 24;
    static std::array<std::byte, 4> lengthToBytes(uint32_t length);
    static uint32_t bytesToLength(std::array<std::byte, 4> bytes);

    void handleRobotRead(SocketConnection& socket, uint32_t message_length);

    void handleVisionRead(SocketConnection& socket, uint32_t message_length);

    void handleSimulationRead(SocketConnection& socket, uint32_t message_length);

    void onRobotCommand(const RobotControl& control, TeamColor color) override;

    void onSimulationCommand(const LuhsoccerSimulatorControl& control) override;

    void printSimulationError(const SimulatorError& error);

    void connectSocket(ErforceSimulationConnector::SocketConnection& socket);

    void handleHeaderRead(ErforceSimulationConnector::SocketConnection& socket);

    void handleSizedRead(ErforceSimulationConnector::SocketConnection& socket, uint32_t length_to_read);

    void openRobotSocket(TeamColor color);
    void closeRobotSocket(TeamColor color);

    TeamColor current_controlling;
    asio::io_context context{};
    asio::ip::tcp::resolver resolver{context};

    constexpr static int SIMULATION_VISION_PORT = 10020;
    constexpr static int SIMULATION_CONTROL_PORT = 10300;
    constexpr static int SIMULATION_CONTROL_BLUE_PORT = 10301;
    constexpr static int SIMULATION_CONTROL_YELLOW_PORT = 10302;

    SocketConnection vision_connection{"localhost", std::to_string(SIMULATION_VISION_PORT), context,
                                       &ErforceSimulationConnector::handleVisionRead};
    SocketConnection blue_robot_connection{"localhost", std::to_string(SIMULATION_CONTROL_BLUE_PORT), context,
                                           &ErforceSimulationConnector::handleRobotRead};
    SocketConnection yellow_robot_connection{"localhost", std::to_string(SIMULATION_CONTROL_YELLOW_PORT), context,
                                             &ErforceSimulationConnector::handleRobotRead};
    SocketConnection simulation_connection{"localhost", std::to_string(SIMULATION_CONTROL_PORT), context,
                                           &ErforceSimulationConnector::handleSimulationRead};

    logger::Logger logger{"er_sim_connector"};
    time::Rate rate{100.0, "ErForceSimulationConnection"};  // Frequency capped at 100 Hz to simulate a real camera??
};

}  // namespace luhsoccer::simulation