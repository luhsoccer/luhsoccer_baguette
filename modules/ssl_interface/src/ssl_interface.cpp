#include "ssl_interface/ssl_interface.hpp"
#include <algorithm>
#include <utility>
#include <vector>
#include "asio/io_context.hpp"
#include "logger/logger.hpp"
#include "vision_multicast/vision_multicast.hpp"
#include "gc_multicast/gc_multicast.hpp"
#include "gc_tcp/gc_tcp.hpp"
#include "ssl_interface/log_file.hpp"
#include "ssl_types_converter/ssl_types_converter.hpp"
#include "simulation_interface/simulation_interface.hpp"
#include "config_provider/config_store_main.hpp"

namespace luhsoccer::ssl_interface {

std::ostream& operator<<(std::ostream& os, const VisionPublishMode& mode) {
    switch (mode) {
        case VisionPublishMode::DISABLED:
            os << "Disabled";
            break;
        case VisionPublishMode::NETWORK:
            os << "Network";
            break;
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const VisionDataSource& source) {
    switch (source) {
        case VisionDataSource::DISABLED:
            os << "Disabled";
            break;
        case VisionDataSource::NETWORK:
            os << "Network";
            break;
        case VisionDataSource::SIMULATION:
            os << "Simulation";
            break;
        case VisionDataSource::GAME_LOG:
            os << "Game-Log";
            break;
    }
    return os;
}

std::ostream& operator<<(std::ostream& os, const GameControllerDataSource& source) {
    switch (source) {
        case GameControllerDataSource::DISABLED:
            os << "Disabled";
            break;
        case GameControllerDataSource::NETWORK:
            os << "Network";
            break;
        case GameControllerDataSource::INTERNAL:
            os << "Internal";
            break;
        case GameControllerDataSource::GAME_LOG:
            os << "Game-Log";
            break;
    }
    return os;
}

class SSLInterface::IoBackend {
    asio::io_context context{};
    connection::VisionMulticastConnection vision_mc;
    connection::GCMulticastConnection gc_mc;
    connection::GCTcpConnection gc_tcp;

   public:
    IoBackend(SSLInterface& interface, config_provider::ConfigStore& config)
        : vision_mc(interface, context, config.ssl_interface_config.vision_ip, config.ssl_interface_config.vision_port),
          gc_mc(interface, context, config.ssl_interface_config.gc_ip, config.ssl_interface_config.gc_port),
          gc_tcp(interface, context){};

    void setup() {
        // try to connect to the game controller via multicast by default
        gc_mc.setup();
        gc_mc.read();

        vision_mc.setup();
        vision_mc.read();
    }

    /**
     * @brief Reconnect to the Game Controller when the Game Controller Data source changes
     *
     * @param gc_source The new Game Controller Data Source
     */
    void reconnectGC(const GameControllerDataSource& gc_source) {
        this->gc_tcp.close();
        this->gc_mc.close();

        switch (gc_source) {
            case GameControllerDataSource::INTERNAL: {
                this->gc_tcp.setup();
                this->gc_tcp.read();
                break;
            }

            case GameControllerDataSource::NETWORK: {
                this->gc_mc.setup();
                this->gc_mc.read();
                break;
            }
        }
    }

    void run() { context.run(); }

    void stop() {
        vision_mc.close();
        gc_mc.close();
        gc_tcp.close();
        context.stop();
    }

    void publish(const SSLWrapperData& data) { vision_mc.publish(data); }
};

SSLInterface::SSLInterface(simulation_interface::SimulationInterface& simulation_interface)
    : io_backend(std::make_unique<IoBackend>(*this, config_provider::ConfigProvider::getConfigStore())),
      simulation_interface(simulation_interface) {}
SSLInterface::~SSLInterface() = default;

void SSLInterface::setup() {
    this->io_backend->setup();
    simulation_interface.setVisionOutput([this](auto&& packet) {
        this->processWrapperData<VisionDataSource::SIMULATION>(
            converter::parseWrapperData(std::forward<decltype(packet)>(packet)));
    });
}

void SSLInterface::setGameControllerDataSource(const GameControllerDataSource source) {
    LOG_INFO(this->logger, "Change game controller data source from {} to {}", this->gamecontroller_source, source);
    this->gamecontroller_source = source;
    this->io_backend->reconnectGC(source);
}

void SSLInterface::loop(std::atomic_bool& should_run) {
    LOG_INFO(logger, "Starting polling all interfaces...");
    while (should_run) {
        this->io_backend->run();
    }
}

void SSLInterface::stop() { this->io_backend->stop(); }

void SSLInterface::publish(const SSLWrapperData& data) { this->io_backend->publish(data); }

void SSLInterface::handlePythonCallbacks(const SSLVisionData& data) {
    for (const auto& ball_callback : this->ball_callbacks) {
        for (const auto& ball : data.balls) {
            ball_callback({ball.position.x(), ball.position.y()}, data.timestamp_capture.asNSec());
        }
    }
    for (const auto& robot_callback : this->robot_callbacks) {
        for (const auto& robot : data.blue_robots) {
            robot_callback({robot.transform.translation().x(), robot.transform.translation().y()}, robot.id, true,
                           data.timestamp_capture.asNSec());
        }
        for (const auto& robot : data.yellow_robots) {
            robot_callback({robot.transform.translation().x(), robot.transform.translation().y()}, robot.id, false,
                           data.timestamp_capture.asNSec());
        }
    }
}

}  // namespace luhsoccer::ssl_interface