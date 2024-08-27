#pragma once

#include <utility>

#include "simulation_interface/simulation_connector.hpp"
#include "logger/logger.hpp"
#include "config_provider/config_store_main.hpp"
#include "config/simulation_interface_config.hpp"
#include "event_system/event_system.hpp"
#include "asio.hpp"

class SimulatorError;

namespace luhsoccer::simulation {
class ErforceSimulationConnector : public SimulationConnector {
   public:
    ErforceSimulationConnector(event_system::EventSystem& event_system, VisionOutputCallback& vision_output,
                               RobotOutputCallback& robot_feedback_output,
                               SimulationOutputCallback& simulation_feedback_output);

    [[nodiscard]] SimulationConnectorType type() const override { return SimulationConnectorType::ERFORCE_SIMULATION; };

    void load() override;

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
    event_system::EventSystem& event_system;
    asio::ip::tcp::resolver resolver;

    SocketConnection vision_connection;

    SocketConnection blue_robot_connection;

    SocketConnection yellow_robot_connection;

    SocketConnection simulation_connection;

    logger::Logger logger{"er_sim_connector"};
};

}  // namespace luhsoccer::simulation