#pragma once

#include "core/module.hpp"
#include "vision_processor/vision_processor_events.hpp"
#include "ssl_interface/ssl_types.hpp"

namespace luhsoccer::vision_processor {

using namespace std::chrono_literals;

/**
 * @brief Processes the SSL-Vision Packets right out of the SSL-interface.
 * This Processor will merge images from multiple cameras into a single one.
 */
class VisionProcessor : public BaguetteModule {
   public:
    VisionProcessor() = default;
    virtual ~VisionProcessor() = default;
    VisionProcessor(const VisionProcessor& processor) = delete;
    VisionProcessor(const VisionProcessor&& processor) = delete;
    VisionProcessor& operator=(const VisionProcessor& processor) = delete;
    VisionProcessor& operator=(const VisionProcessor&& processor) = delete;

   public:
    void setup([[maybe_unused]] event_system::EventSystem& event_system) override;

    /**
     * @brief Invalidates vision data that is older than DATA_MAX_AGE_MS
     */
    void invalidateVisionData();

    /**
     * @brief Invalidates the latest Truth if it has gotten too old.
     * This is so that if we stop receiving vision packets for a certain duration
     * and then start receiving them again, we accept the new data instead of saying
     * it is not playusible
     */
    void invalidateLatestTruth();

    /**
     * @brief Checks wether the new ProcessedVisionData is plausible and overwrites the implausable data
     * e.g. if the position of the ball could be true according to a certain max ball velocity
     *
     * @param new_data The new ProcessedVisionData and the corresponding time
     */
    void checkForUpdatePlausability(ProcessedVisionData& new_data);

    /**
     * @brief Checks if the Currently stored vision data are plausable against the latest truth
     * Take the Id of the cam to check the data from, so that only the new data is checked
     */
    void checkVisionDataPlausability(size_t cam_index);

    /**
     * @brief
     *
     * @param cam_id The Camera ID of the cam which last sent an update
     * @return ProcessedVisionData
     */
    ProcessedVisionData mergeVisionData(size_t cam_id);

    void handleTeleport(const TeleportData& data);

    bool wasRecentlyTeleported(size_t id, TeamColor team, time::TimePoint time);
    bool wasBallRecentlyTeleported(time::TimePoint time);

    void clearRecentTeleports(time::TimePoint time);

    constexpr std::string_view moduleName() override { return "vision_processor"; }

   private:
    std::unordered_map<size_t, std::pair<ssl_interface::SSLVisionData, time::TimePoint>> last_cam_data;
    std::vector<TeleportData> recent_teleports;
    std::optional<ProcessedVisionData> latest_truth;
    mutable std::mutex teleport_mtx;

    static constexpr time::Duration DATA_MAX_AGE{250ms};
    static constexpr time::Duration TELEPORT_TIMEOUT{0.5};
};

}  // namespace luhsoccer::vision_processor