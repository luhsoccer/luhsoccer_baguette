#include "ssl_interface/ssl_interface.hpp"
#include <utility>
#include "logger/logger.hpp"
#include "vision_multicast/vision_multicast.hpp"
#include "gc_multicast/gc_multicast.hpp"
#include "gc_tcp/gc_tcp.hpp"
#include "ssl_interface/log_file.hpp"
#include "ssl_types_converter/ssl_types_converter.hpp"
#include "simulation_interface/simulation_interface.hpp"
#include "config_provider/config_store_main.hpp"
#include "config/ssl_interface_config.hpp"

namespace luhsoccer::ssl_interface {

std::string_view format_as(const VisionPublishMode& mode) {
    switch (mode) {
        case VisionPublishMode::DISABLED:
            return "Disabled";
        case VisionPublishMode::NETWORK:
            return "Network";
    }
};

std::string_view format_as(const VisionDataSource& source) {
    switch (source) {
        case VisionDataSource::DISABLED:
            return "Disabled";
        case VisionDataSource::NETWORK:
            return "Network";
        case VisionDataSource::SIMULATION:
            return "Simulation";
        case VisionDataSource::GAME_LOG:
            return "Game-Log";
    }
}

std::string_view format_as(const GameControllerDataSource& source) {
    switch (source) {
        case GameControllerDataSource::DISABLED:
            return "Disabled";
        case GameControllerDataSource::NETWORK:
            return "Network";
        case GameControllerDataSource::INTERNAL:
            return "Internal";
        case GameControllerDataSource::GAME_LOG:
            return "Game-Log";
    }
}

class SSLInterface::IoBackend {
    connection::VisionMulticastConnection vision_mc;
    connection::GCMulticastConnection gc_mc;
    connection::GCTcpConnection gc_tcp;

   public:
    IoBackend(SSLInterface& interface, event_system::EventSystem& event_system, config_provider::ConfigStore& config)
        : vision_mc(interface, event_system, config.ssl_interface_config.vision_ip,
                    config.ssl_interface_config.vision_port),
          gc_mc(interface, event_system, config.ssl_interface_config.gc_ip, config.ssl_interface_config.gc_port),
          gc_tcp(interface, event_system){};

    void setup() {
        // try to connect to the game controller via multicast by default
        gc_mc.setup();
        gc_mc.read();

        gc_tcp.setup();

        vision_mc.setup();
        vision_mc.read();
    }

    void stop() {
        vision_mc.close();
        gc_mc.close();
        gc_tcp.close();
    }

    void publish(const SSLWrapperData& data) { vision_mc.publish(data); }
};

SSLInterface::SSLInterface(simulation_interface::SimulationInterface& simulation_interface,
                           event_system::EventSystem& event_system, marker::MarkerService& ms)
    : io_backend(std::make_unique<IoBackend>(*this, event_system, config_provider::ConfigProvider::getConfigStore())),
      simulation_interface(simulation_interface),
      event_system(event_system),
      ms(ms) {}

SSLInterface::~SSLInterface() = default;

void SSLInterface::setup() {
    this->io_backend->setup();
    simulation_interface.setVisionOutput([this](auto&& packet) {
        this->processWrapperData<VisionDataSource::SIMULATION>(
            converter::parseWrapperData(std::forward<decltype(packet)>(packet)));
    });
}

void SSLInterface::setGameControllerDataSource(const GameControllerDataSource source) {
    this->logger.info("Change game controller data source from {} to {}", this->gamecontroller_source, source);
    this->gamecontroller_source = source;
}

void SSLInterface::stop() { this->io_backend->stop(); }

void SSLInterface::publish(const SSLWrapperData& data) { this->io_backend->publish(data); }

}  // namespace luhsoccer::ssl_interface