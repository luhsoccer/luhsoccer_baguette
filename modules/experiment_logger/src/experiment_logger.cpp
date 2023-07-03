#include "experiment_logger/experiment_logger.hpp"
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include "marker_service/marker_service.hpp"
#include "utils/utils.hpp"

namespace luhsoccer::experiment_logger {

ExperimentLogger::ExperimentLogger(marker::MarkerService& ms, const std::string& log_folder,
                                   const std::string& index_file)
    : ms(ms),
      experiment_running(false),
      log_folder(getBaguetteDirectory().append(log_folder)),
      index_file(getBaguetteDirectory().append(log_folder).append(index_file + ".csv")) {
    if (!std::filesystem::exists(this->log_folder)) std::filesystem::create_directory(this->log_folder);
};

bool ExperimentLogger::startExperiment(std::string name, std::string experiment_folder_name,
                                       const std::shared_ptr<const transform::WorldModel>& wm,
                                       const std::vector<TrackTarget>& track_targets,
                                       const std::vector<transform::Position>& obstacles, bool include_date) {
    if (experiment_running) return false;
    experiment_running = true;
    this->experiment_data = ExperimentLogger::ExperimentData();
    time_t t = std::time(nullptr);
    tm tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d--%H-%M-%S");
    this->experiment_data.date = oss.str();
    if (include_date) {
        name += "--" + this->experiment_data.date;
    }
    std::replace(name.begin(), name.end(), '_', '-');
    std::replace(experiment_folder_name.begin(), experiment_folder_name.end(), '_', '-');

    this->experiment_data.name = name;
    this->experiment_data.experiment_folder = experiment_folder_name;
    this->experiment_data.wm = wm;
    this->experiment_data.track_targets = track_targets;
    for (const auto& track : track_targets) {
        auto transform = track.goal.getCurrentPosition(wm);
        if (transform.has_value()) this->experiment_data.track_goal_points.push_back(transform.value());
    }

    for (const auto& obstacle_position : obstacles) {
        auto transform = obstacle_position.getCurrentPosition(wm);
        if (transform.has_value()) this->experiment_data.obstacles.push_back(transform.value());
    }

    std::filesystem::path experiment_folder = log_folder;
    experiment_folder.append(experiment_folder_name);
    if (!std::filesystem::exists(experiment_folder)) std::filesystem::create_directory(experiment_folder);

    std::fstream index_file_stream;
    index_file_stream.open(std::filesystem::absolute(this->index_file).string(), std::ios_base::app);
    index_file_stream << experiment_folder_name << "," << name << "\n";
    index_file_stream.close();

    this->experiment_data.experiment_file = experiment_folder;
    this->experiment_data.experiment_file.append(name + ".csv");

    this->experiment_data.marker_points.resize(track_targets.size());

    return true;
}

bool ExperimentLogger::endExperiment(bool discard) {
    if (!this->experiment_running) return false;
    this->experiment_running = false;
    LOG_INFO(this->logger, "Experiment stoped recorded {:d} data points on {:d} tracks",
             this->experiment_data.data.size(),
             this->experiment_data.data.size() > 0 ? this->experiment_data.data[0].size() : 0);
    if (this->experiment_data.data.size() < 2 || this->experiment_data.data[0].size() < 1) return false;
    if (!discard) {
        // analyze path
        time::Duration time_for_path =
            this->experiment_data.data.back().at(0).header.stamp - this->experiment_data.data[0][0].header.stamp;

        double length = 0.0;

        double average_velocity = 0.0;

        double time_to_optimal_path = 0.0;

        // write results
        std::ofstream result_stream;
        result_stream.open(std::filesystem::absolute(this->experiment_data.experiment_file).string());

        result_stream << "Metadata\n";
        result_stream << "Date,Time for path in s,Length of path in m,Average velocity in m/s,time to optimal path\n";
        result_stream << fmt::format("{},{:0.6f},{:0.6f},{:0.6f},{:0.6f}\n", this->experiment_data.date,
                                     time_for_path.asSec(), length, average_velocity, time_to_optimal_path);
        result_stream << "Objects\n";
        result_stream << "X,Y,Heading\n";
        for (const auto& obstacle : this->experiment_data.obstacles) {
            result_stream << obstacle.translation().x() << "," << obstacle.translation().y() << ","
                          << Eigen::Rotation2Dd(obstacle.rotation()).angle() << "\n";
        }

        result_stream << "StartEndPoint\n";
        result_stream << "X,Y,Heading,X,Y,Heading\n";
        if (this->experiment_data.track_goal_points.size() != this->experiment_data.data[0].size() ||
            this->experiment_data.track_targets.size() != this->experiment_data.data[0].size())
            throw std::runtime_error("Sizes do not match");

        for (size_t i = 0; i < this->experiment_data.track_targets.size(); i++) {
            const auto& start = this->experiment_data.data[0][i].transform;
            const auto& goal = this->experiment_data.track_goal_points[0];
            if (start.has_value()) {
                result_stream << start->translation().x() << "," << start->translation().y() << ","
                              << Eigen::Rotation2Dd(start->rotation()).angle() << "," << goal.translation().x() << ","
                              << goal.translation().y() << "," << Eigen::Rotation2Dd(goal.rotation()).angle() << "\n";
            }
        }
        result_stream << "Path\n";

        for (size_t i = 0; i < this->experiment_data.track_targets.size(); i++) {
            result_stream << "X,Y,Heading,Velocity,";
        }
        result_stream << "\n";

        for (const auto& data_point : this->experiment_data.data) {
            for (const auto& track_data : data_point) {
                if (track_data.transform) {
                    result_stream << track_data.transform->translation().x() << ","
                                  << track_data.transform->translation().y() << ","
                                  << Eigen::Rotation2Dd(track_data.transform->rotation()).angle();
                } else {
                    result_stream << "0.0,0.0,0.0";
                }
                if (track_data.velocity) {
                    result_stream << "," << track_data.velocity->head(2).norm() << ",";
                } else {
                    result_stream << ",0.0,";
                }
            }
            result_stream << "\n";
        }
        result_stream.close();
        LOG_INFO(this->logger, "Experiment with name '{}' was saved to file {}!", this->experiment_data.name,
                 std::filesystem::absolute(this->experiment_data.experiment_file).string());
    }

    // cleanup
    this->experiment_data = ExperimentLogger::ExperimentData();
    return true;
};

bool ExperimentLogger::newDataPoint(const time::TimePoint& time, bool viz_result) {
    if (!this->experiment_running) return false;
    time::TimePoint stamp = time;
    if (time.asSec() == 0.0) stamp = time::now();
    std::vector<transform::TransformWithVelocity> data_point;

    int id = 0;
    auto points_it = this->experiment_data.marker_points.begin();
    for (const auto& track : this->experiment_data.track_targets) {
        transform::TransformWithVelocity track_data;
        track_data.transform = track.position.getCurrentPosition(this->experiment_data.wm, track.global_position, time);
        track_data.velocity =
            track.position.getVelocity(this->experiment_data.wm, track.global_position, track.global_position, time);
        track_data.header.stamp = stamp;
        data_point.push_back(track_data);

        if (track_data.transform.has_value()) {
            points_it->push_back(
                {track_data.transform->translation().x(), track_data.transform->translation().y(), 0.0});
        }

        if (viz_result && points_it->size() > 1) {
            marker::LineStrip marker(track.global_position, "experiment_log", id++);
            marker.setColor(marker::Color::RED());
            marker.setPathClosed(false);
            marker.setPoints(*points_it);
            constexpr double EXP_MARKER_LIFETIME = 10.0;
            marker.setLifetime(EXP_MARKER_LIFETIME);
            this->ms.displayMarker(marker);
        }
        points_it++;
    }
    this->experiment_data.data.push_back(data_point);
    return true;
}
}  // namespace luhsoccer::experiment_logger
