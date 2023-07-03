#pragma once

#include <filesystem>
#include <string>
#include <Eigen/Geometry>

#include "robot_identifier.hpp"
#include "transform/transform.hpp"
#include "marker_service/marker.hpp"

namespace luhsoccer {

namespace marker {
class MarkerService;
}
namespace experiment_logger {

struct TrackTarget {
    std::string name;
    transform::Position position;
    transform::Position goal;
    transform::Position global_position;
};

class ExperimentLogger {
   public:
    ExperimentLogger(marker::MarkerService& ms, const std::string& log_folder,
                     const std::string& index_file = "ba-experiments");

    bool startExperiment(std::string name, std::string experiment_folder_name,
                         const std::shared_ptr<const transform::WorldModel>& wm,
                         const std::vector<TrackTarget>& track_targets,
                         const std::vector<transform::Position>& obstacles, bool include_date = true);

    bool newDataPoint(const time::TimePoint& time = time::TimePoint(0), bool viz_result = true);
    bool endExperiment(bool discard = false);

   private:
    marker::MarkerService& ms;
    std::atomic_bool experiment_running;
    std::filesystem::path log_folder;
    std::filesystem::path index_file;
    struct ExperimentData {
        std::string name;
        std::string experiment_folder;
        std::filesystem::path experiment_file;
        std::shared_ptr<const transform::WorldModel> wm;
        std::vector<TrackTarget> track_targets;
        std::vector<Eigen::Affine2d> track_goal_points;
        std::vector<Eigen::Affine2d> obstacles;
        std::string date;

        std::vector<std::vector<transform::TransformWithVelocity>> data;

        std::vector<std::vector<marker::Point>> marker_points;
    };
    ExperimentData experiment_data;

    logger::Logger logger{"experiment_logger"};
};
}  // namespace experiment_logger
}  // namespace luhsoccer