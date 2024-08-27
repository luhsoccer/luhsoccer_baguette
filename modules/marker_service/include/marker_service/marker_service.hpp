#pragma once

#include <map>
#include "core/module.hpp"
#include "logger/logger.hpp"
#include "time/time.hpp"
#include "transform/transform.hpp"
#include "marker.hpp"
#include <mutex>

namespace luhsoccer::marker {

class MarkerImpl;
class Marker2DImpl;

struct LuhvizMarkers;

/**
 * @brief provides the function for other modules to create and display markers
 * in luhviz's renderview
 *
 */
class MarkerService : public BaguetteModule {
   public:
    virtual ~MarkerService();
    MarkerService(MarkerService&&) = delete;
    MarkerService(MarkerService&) = delete;
    MarkerService& operator=(MarkerService&&) = delete;
    MarkerService& operator=(MarkerService&) = delete;
    /**
     * @brief Construct a new Marker Service object
     *
     * @param wm reference to the worldmodel
     */
    MarkerService();

    /**
     * @brief Set the Real Worldmodel object
     *
     * @param wm
     */
    void setRealWorldmodel(std::shared_ptr<const transform::WorldModel> wm) { this->real_wm = wm; }

    /**
     * @brief adds the given marker to the markers map or updates it if its already present
     *
     * @param marker
     */
    void displayMarker(Marker marker);

    /**
     * @brief adds the given marker to the markers map or updates it if its already present
     *
     * @param marker
     */
    void displayMarker(RobotInfo marker);

    void removeRobotInfoMarker(RobotIdentifier robot);

    /**
     * @brief adds the given marker to the markers map or updates it if its already present
     *
     * @param marker
     */
    void displayMarker(Info marker);

    /**
     * @brief adds the given marker to the markers map or updates it if its already present
     *
     * @param marker
     */
    void displayMarker(LinePlot plot);

    /**
     * @brief removes a marker if it is present
     *
     */
    void deleteMarker(std::string ns, size_t id);

    /**
     * @brief Updates the transforms and returns the luhviz markers
     *
     * @return LuhvizMarkers
     */
    std::unique_ptr<LuhvizMarkers> getLuhvizMarkers();

    /**
     * @brief Get the Key object
     *
     * @param ns
     * @param id
     * @return std::string
     */
    std::string getKey(const std::string& ns, size_t id);

    /**
     * @brief converts the marker type to type2d for luhviz
     *
     * @param type
     * @return MarkerImpl::Type2D
     */
    Type2D get2DMarkerType(MType type) {
        switch (type) {
            case MType::LINE:
                return Type2D::LINE2D;
            case MType::RECT:
                return Type2D::RECT2D;
            case MType::ARROW_2D:
                return Type2D::ARROW2D;
            case MType::CIRCLE:
                return Type2D::CIRCLE2D;
            case MType::LINE_STRIP:
                return Type2D::LINE_STRIP2D;
            case MType::CUSTOM_STRIP:
                return Type2D::CUSTOM_STRIP2D;
            case MType::HEATMAP:
                return Type2D::HEATMAP2D;
            case MType::CIRCULAR_HEATMAP:
                return Type2D::CIRCULAR_HEATMAP2D;
            default:
                LOGGER.warning("Could not convert 3D MarkerType to 2D MarkerType");
                break;
        }
        return Type2D::LAST_MARKER_TYPE2D;
    }

    /**
     * @brief converts the marker type to type3d for luhviz
     *
     * @param type
     * @return MarkerImpl::Type2D
     */
    Type3D get3DMarkerType(MType type) {
        switch (type) {
            case MType::GOAL_BORDERS_DIVA:
                return Type3D::GOAL_BORDERS3D_DIVA;
            case MType::GOAL_BORDERS_DIVB:
                return Type3D::GOAL_BORDERS3D_DIVB;
            case MType::ROBOT:
                return Type3D::ROBOT3D;
            case MType::BALL:
                return Type3D::BALL3D;
            case MType::FRAME:
                return Type3D::FRAME3D;
            case MType::CONE:
                return Type3D::CONE3D;
            case MType::CUBE:
                return Type3D::CUBE3D;
            case MType::CYLINDER:
                return Type3D::CYLINDER3D;
            case MType::SPHERE:
                return Type3D::SPHERE3D;
            case MType::TORUS:
                return Type3D::TORUS3D;
            case MType::ARROW:
                return Type3D::ARROW3D;
            case MType::SUZANNE:
                return Type3D::SUZANNE3D;
            case MType::TEXT:
                return Type3D::TEXT3D;
            default:
                LOGGER.warning("Could not convert 2D MarkerType to 3D MarkerType");
                break;
        }
        return Type3D::LAST_MARKER_TYPE3D;
    }

    void setup() override;
    void stop() override;
    std::string_view moduleName() override { return "marker_service"; }

   private:
    /** logger for logging */
    const static logger::Logger LOGGER;

    /** the worldmodel reference */
    std::optional<std::shared_ptr<const transform::WorldModel>> real_wm{std::nullopt};

    /** for locking of the map while accessing it */
    mutable std::mutex marker_mutex{};

    /** holds the markers to display as a map with key "namespace_id" */
    std::unordered_map<std::string, Marker> markers;

    /** holds the robot infos (special marker type) */
    std::unordered_map<std::string, RobotInfo> robot_infos;
    mutable std::mutex robot_info_mutex{};

    /** holds the lineplot infos (special marker type) */
    std::unordered_map<std::string, LinePlot> line_plots;
    mutable std::mutex line_plots_mutex{};

    /** holds other infos */
    std::map<std::string, std::map<std::string, Info>> infos;
    mutable std::mutex info_mutex{};

    /** for locking of the luhviz_markers while accessing it */
    mutable std::mutex luhviz_marker_mutex{};
    std::unique_ptr<LuhvizMarkers> luhviz_markers;

    /**
     * @brief converts a given marker into ints MarkerImpl version which luhviz needs to display it correctly
     *
     * @param marker the marker to display
     * @param tf the tf of the marker retrieved from worldmodel
     * @param markers struct to append the created MarkerImpl
     */
    void convertMarkerForLuhviz(Marker& marker, Eigen::Affine2d tf, const transform::Velocity& vel);

    /**
     * @brief Creates a MarkerImpl of the given type for displaying in luhviz this is used to display all available tfs
     * from worldmodel
     *
     * @param type the markertype
     * @param tf the tf from worldmodel
     * @param markers struct to append the MarkerImpl
     */
    void createFrameMarkerForLuhviz(const transform::Transform& tf, size_t index);

    /**
     * @brief creates a marker for every char from the text marker and adds them to the list
     *
     * @param marker the text marker to split into chars
     * @param markers ref to luhvizMarkers struct
     * @return MarkerImpl
     */
    void convertTextMarkerForLuhviz(Eigen::Vector3d position, double angle, Marker& marker);

    /**
     * @brief update the transforms from the worldmodel and retrieve all updated marker objects as vector (called by
     * luhviz at approx. 60hz)
     *
     */
    void updateTransforms();
};
}  // namespace luhsoccer::marker
