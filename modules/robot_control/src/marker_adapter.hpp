#pragma once

#include "marker_service/marker.hpp"

namespace luhsoccer::marker {
class MarkerService;

}  // namespace luhsoccer::marker

namespace luhsoccer::robot_control {

class MarkerAdapter {
   public:
    explicit MarkerAdapter(marker::MarkerService& ms) : parent_type(ParentAdapterType::MARKER_SERVICE), ms(&ms) {}
    MarkerAdapter(const MarkerAdapter& ma) : parent_type(ParentAdapterType::MARKER_ADAPTER), ma(&ma) {}
    MarkerAdapter() : parent_type(ParentAdapterType::NONE) {}

    void setOverrideColor(const marker::Color& color) { this->color = color; }
    void setOverrideNamespace(const std::string& ns) { this->ns = ns; }
    void setOverrideOccupancy(double occupancy) { this->occupancy = occupancy; }
    void setOverrideLifetime(double lifetime) { this->lifetime = lifetime; }

    void setManageIDs() { this->manage_ids = true; }
    void resetIDs() { this->last_id = 0; }

    MarkerAdapter(MarkerAdapter&&) = delete;
    MarkerAdapter& operator=(const MarkerAdapter&) = delete;
    MarkerAdapter& operator=(MarkerAdapter&&) = delete;
    ~MarkerAdapter() = default;

    /**
     * @brief adds the given marker to the markers map or updates it if its already present
     *
     * @param marker
     */
    void displayMarker(marker::Marker marker) const;

    /**
     * @brief adds the given marker to the markers map or updates it if its already present
     *
     * @param marker
     */
    void displayMarker(const marker::RobotInfo& marker) const;

    /**
     * @brief adds the given marker to the markers map or updates it if its already present
     *
     * @param marker
     */
    void displayMarker(const marker::Info& marker) const;

    /**
     * @brief adds the given marker to the markers map or updates it if its already present
     *
     * @param marker
     */
    void displayMarker(const marker::LinePlot& plot) const;

    /**
     * @brief removes a marker if it is present
     *
     */
    void deleteMarker(std::string ns, size_t id) const;

   private:
    void overrideMarker(marker::Marker& marker) const;

    enum class ParentAdapterType { MARKER_SERVICE, MARKER_ADAPTER, NONE } parent_type;

    marker::MarkerService* ms{nullptr};
    const MarkerAdapter* ma{nullptr};

    std::optional<marker::Color> color;
    std::optional<std::string> ns;
    std::optional<double> occupancy;
    std::optional<double> lifetime;
    bool manage_ids{false};
    mutable size_t last_id{0};
};
}  // namespace luhsoccer::robot_control