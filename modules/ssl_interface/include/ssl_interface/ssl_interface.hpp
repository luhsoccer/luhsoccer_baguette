#pragma once

#include <functional>
#include <memory>
#include <utility>
#include <vector>
#include <mutex>
#include <optional>
#include "logger/logger.hpp"
#include "module.hpp"
#include "ssl_interface/ssl_types.hpp"

namespace luhsoccer::simulation_interface {
class SimulationInterface;
}

namespace luhsoccer::ssl_interface {

/**
 * @brief Changes the mode of the ssl interface
 *
 */
enum class VisionPublishMode {
    /// Disable any vision data publishing
    DISABLED,
    /// Publish the data from the current vision source to the network.
    NETWORK,
};

std::ostream& operator<<(std::ostream& os, const VisionPublishMode& mode);

/**
 * @brief The source of the data that will be published to other modules
 *
 */
enum class VisionDataSource {
    /// Disable every external input. Could be used for simulate a vision blackout
    DISABLED,
    /// Read the data from the network protocol. In the most cases this is an external ssl-vision instance
    NETWORK,
    /// Read the data from the simulation contained in this software
    SIMULATION,
    /// Replay the data from a game log
    GAME_LOG,
};

std::ostream& operator<<(std::ostream& os, const VisionDataSource& source);

enum class GameControllerDataSource {
    /// Disable the game controller completely
    DISABLED,
    /// Use the external game controller from the network
    NETWORK,
    /// Use the integrated game controller in the software
    INTERNAL,
    /// Use the gamecontroller messages from
    GAME_LOG,
};

std::ostream& operator<<(std::ostream& os, const GameControllerDataSource& source);

/**
 * @brief Class for interacting with ssl software suite.
 *  This class setup all handlers that are used to communication with all ssl tools or an simulation instance.a
 */
class SSLInterface : public BaguetteModule {
   public:
    explicit SSLInterface(simulation_interface::SimulationInterface& simulation_interface);
    virtual ~SSLInterface();
    SSLInterface(SSLInterface&&) = delete;
    SSLInterface(SSLInterface&) = delete;
    SSLInterface& operator=(SSLInterface&&) = delete;
    SSLInterface& operator=(SSLInterface&) = delete;

    /**
     * @brief Setup all network interfaces and endpoints.
     * Should be called in the same thread as the other functions.
     */
    void setup() override;

    /**
     * @brief polls on every interface and listens for data
     * Function is blocking and a return indicates that all interfaces are closed or have failed.
     */
    void loop(std::atomic_bool& should_run) override;

    void stop() override;

    /**
     * @brief publish new data from a wrapper to the current vision listener and/or the current field data listener (in
     * almost all cases this should be the GameDataProvider). The data is only published if the source matches the
     * current selected source.
     *
     * @tparam source the source of the data
     * @param data the new wrapper data
     */
    template <enum VisionDataSource source>
    void processWrapperData(SSLWrapperData data) {
        // Fail when the vision source does not exists
        static_assert(source != VisionDataSource::DISABLED, "Can't publish with DISABLED as source");
        // Only publish the data if it is from the right source.
        // This way no mixing of data sources can happen
        if (source == this->vision_source) {
            // Publish vision data when requested
            if (this->vision_publish_mode == VisionPublishMode::NETWORK) {
                if (source != VisionDataSource::NETWORK) {
                    this->publish(data);
                } else {  // Don't publish when we're receiving from the network. That would result in a loop
                    LOG_WARNING(logger,
                                "VisionPublishMode is set to {} but VisionDataSource is also set to {}. That would "
                                "result in a loop!",
                                vision_publish_mode, vision_publish_mode);
                }
            }

            // Check if the wrapper packet contains a vision and/or a geometry packet
            if (data.vision) {
                handlePythonCallbacks(data.vision.value());
                this->vision_callback(std::move(data.vision.value()));
            }
            if (data.field) {
                this->field_callback(std::move(data.field.value()));
            }
        }
    }

    /**
     * @brief publish new game controller data to the current listener (in almost all cases this should be the
     * GameDataProvider). The data is only published if the source matches the current selected source.
     *
     * @tparam the source of the data
     * @param data the new vision data
     */
    template <enum GameControllerDataSource source>
    void processGameControllerData(SSLGameControllerData data) {
        // Fail when the source does not exists
        static_assert(source != GameControllerDataSource::DISABLED, "Can't publish with DISABLED as source");
        // Publish only when the source is selected
        if (source == this->gamecontroller_source) {
            this->gc_callback(std::move(data));
        }
    }

    /**
     * @brief Set the Vision Callback object
     *
     * @param callback
     */
    void setVisionCallback(std::function<void(SSLVisionData)> callback) { this->vision_callback = std::move(callback); }

    /**
     * @brief Set the game controller callback object
     *
     * @param callback
     */
    void setGameControllerCallback(std::function<void(SSLGameControllerData)> callback) {
        this->gc_callback = std::move(callback);
    }

    /**
     * @brief Sets the mode of the ssl interface. Warning: This could disable the connection to the outer world
     *
     * @param mode the new mode
     */
    void setVisionPublishMode(const VisionPublishMode mode) {
        LOG_INFO(logger, "Change ssl interface mode from {} to {}", this->vision_publish_mode, mode);
        this->vision_publish_mode = mode;
    }

    /**
     * @brief Gets the current vision publish mode
     *
     * @return VisionPublishMode the current vision publish mode
     */
    [[nodiscard]] VisionPublishMode getVisionPublishMode() const { return this->vision_publish_mode; }

    void setVisionDataSource(const VisionDataSource source) {
        LOG_INFO(logger, "Change vision data source from {} to {}", this->vision_source, source);
        this->vision_source = source;
    }

    /**
     * @brief Gets the current vision data source
     *
     * @return VisionDataSource the current vision data source
     */
    [[nodiscard]] VisionDataSource getVisionDataSource() const { return this->vision_source; }

    void setGameControllerDataSource(const GameControllerDataSource source);

    /**
     * @brief Gets the current gamecontroller data source
     *
     * @return GameControllerDataSource the current gamecontroller data source
     */
    [[nodiscard]] GameControllerDataSource getGameControllerDataSource() const { return this->gamecontroller_source; }

    /**
     * @brief Set the Vision Callback object
     *
     * @param callback
     */
    void setFieldCallback(std::function<void(SSLFieldData)> callback) { this->field_callback = std::move(callback); }

    std::string_view moduleName() override { return "ssl_interface"; }

   private:
    void handlePythonCallbacks(const SSLVisionData& data);
    void publish(const SSLWrapperData& data);

    std::function<void(SSLVisionData data)> vision_callback = [](const auto&) {};
    std::function<void(SSLFieldData data)> field_callback = [](const auto&) {};
    std::function<void(SSLGameControllerData data)> gc_callback = [](const auto&) {};

    using BallCallback = std::function<void(Eigen::Vector2d, unsigned long)>;
    using RobotCallback = std::function<void(Eigen::Vector2d, unsigned int, bool, unsigned long)>;

    std::vector<BallCallback> ball_callbacks;
    std::vector<RobotCallback> robot_callbacks;

    VisionPublishMode vision_publish_mode{VisionPublishMode::DISABLED};
    VisionDataSource vision_source{VisionDataSource::NETWORK};
    GameControllerDataSource gamecontroller_source{GameControllerDataSource::DISABLED};

    logger::Logger logger{"ssl_interface"};
    // Hide implementation to remove the direct dependency on the io library.
    class IoBackend;
    std::unique_ptr<IoBackend> io_backend;

    simulation_interface::SimulationInterface& simulation_interface;
};

}  // namespace luhsoccer::ssl_interface