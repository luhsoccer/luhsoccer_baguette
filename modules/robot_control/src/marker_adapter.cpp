#include "marker_adapter.hpp"
#include "marker_service/marker_service.hpp"

namespace luhsoccer::robot_control {
void MarkerAdapter::overrideMarker(marker::Marker& marker) const {
    if (this->color.has_value()) {
        marker.setColor(this->color.value());
    }
    if (this->occupancy.has_value()) {
        auto marker_color = marker.getColor();
        marker_color.alpha = this->occupancy.value();
        marker.setColor(marker_color);
    }
    if (this->ns.has_value()) {
        marker.setNs(this->ns.value());
    }
    if (this->lifetime.has_value()) {
        marker.setLifetime(this->lifetime.value());
    }
    if (this->manage_ids) {
        marker.setId(this->last_id++);
    }
}

// NOLINTNEXTLINE(misc-no-recursion) - recursion is intended
void MarkerAdapter::displayMarker(marker::Marker marker) const {
    this->overrideMarker(marker);

    switch (parent_type) {
        case ParentAdapterType::MARKER_ADAPTER:
            this->ma->displayMarker(marker);
            break;

        case ParentAdapterType::MARKER_SERVICE:
            this->ms->displayMarker(marker);
            break;

        case ParentAdapterType::NONE:
        default:
            break;
    }
}

// NOLINTNEXTLINE(misc-no-recursion) - recursion is intended
void MarkerAdapter::displayMarker(const marker::RobotInfo& marker) const {
    switch (parent_type) {
        case ParentAdapterType::MARKER_ADAPTER:
            this->ma->displayMarker(marker);
            break;

        case ParentAdapterType::MARKER_SERVICE:
            this->ms->displayMarker(marker);
            break;

        default:
            break;
    }
}

// NOLINTNEXTLINE(misc-no-recursion) - recursion is intended
void MarkerAdapter::displayMarker(const marker::Info& marker) const {
    switch (parent_type) {
        case ParentAdapterType::MARKER_ADAPTER:
            this->ma->displayMarker(marker);
            break;

        case ParentAdapterType::MARKER_SERVICE:
            this->ms->displayMarker(marker);
            break;

        default:
            break;
    }
}

// NOLINTNEXTLINE(misc-no-recursion) - recursion is intended
void MarkerAdapter::displayMarker(const marker::LinePlot& plot) const {
    switch (parent_type) {
        case ParentAdapterType::MARKER_ADAPTER:
            this->ma->displayMarker(plot);
            break;

        case ParentAdapterType::MARKER_SERVICE:
            this->ms->displayMarker(plot);
            break;

        default:
            break;
    }
}
}  // namespace luhsoccer::robot_control