
#pragma once

#include "transform/transform.hpp"
#include "robot_interface/robot_interface_types.hpp"

namespace luhsoccer::local_planner {

constexpr size_t DEFAULT_HISTORY_BUFFER_SIZE = 10000;

class AgentHistory {
   public:
    struct Step {
        time::TimePoint stamp;
        Eigen::Affine2d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
        robot_interface::RobotCommand command_msg;
    };

    AgentHistory() = default;

    void clear() {
        this->steps.clear();
        min_obstacle_distance.reset();
    }

    void addStep(const Step& step) { this->steps.push_front(step); };

    void checkMinObstacleDistance(double obstacle_distance) {
        if (!this->min_obstacle_distance || this->min_obstacle_distance.value() < obstacle_distance) {
            this->min_obstacle_distance = obstacle_distance;
        }
    }

    [[nodiscard]] const std::list<Step>& getSteps() const { return this->steps; }

    [[nodiscard]] time::Duration getTrackedDuration() const {
        return this->steps.front().stamp - this->steps.back().stamp;
    }

    [[nodiscard]] std::optional<Step> getStepAtTime(time::TimePoint time) const {
        // check if newest time step is older than requested time
        if (this->steps.front().stamp < time) return std::nullopt;

        auto res = std::find_if(this->steps.begin(), this->steps.end(),
                                [&time](const Step& step) { return step.stamp < time; });

        if (res != this->steps.end()) return *res;

        // if no step is older than the requested time, timepoint is far in past
        return std::nullopt;
    }

    [[nodiscard]] const Step& getLatestStep() const { return this->steps.front(); };
    [[nodiscard]] const Step& getFirstStep() const { return this->steps.back(); };
    [[nodiscard]] std::optional<double> getMinObstacleDistance() const { return this->min_obstacle_distance; };

   private:
    // latest = front, oldest = back
    std::list<Step> steps;
    std::optional<double> min_obstacle_distance;
    transform::Position origin{""};
};
}  // namespace luhsoccer::local_planner