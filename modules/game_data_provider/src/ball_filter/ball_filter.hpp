#pragma once

#include "static_queue.hpp"
#include "robot_identifier.hpp"
#include "ssl_interface/ssl_types.hpp"
#include "transform/transform.hpp"
#include "logger/logger.hpp"
#include "base_filter.hpp"
#include "static_queue.hpp"

namespace luhsoccer::game_data_provider {

class BallFilter {
   public:
    /**
     * @brief Construct a new Ball Filter object
     *
     * @param ball_frame frame_id of the ball
     * @param global_frame frame_id of the global_frame
     */
    explicit BallFilter(std::string ball_frame, std::string global_frame,
                        std::shared_ptr<transform::WorldModel> world_model);

    /**
     * @brief add light barrier feedback of a robot
     *
     * @param robot robot identifier
     * @param has_ball if the robot has the ball in the dribbler
     * @param time time of feedback
     * @return true if the feedback is used
     * @return false if the feedback is discarded
     */
    bool addRobotDribblerStatus(const RobotIdentifier& robot, bool has_ball, [[maybe_unused]] time::TimePoint time);

    /**
     * @brief add vision measurement of the ball
     *
     * @param data ball positions
     * @param time time of the measurement
     * @param camera_id id of the camera
     * @return true if the measurement is used
     * @return false if the measurement is discarded
     */
    bool addVisionData(const std::vector<ssl_interface::SSLBallInfo>& data, time::TimePoint time,
                       const size_t camera_id);

    /**
     * @brief calculate the ball velocity based on multiple old ball positions and the time
     *
     * @param old_ball_position latest ball position
     * @param new_ball_position current ball position
     * @param old_stamp latest time point
     * @param new_stamp current time point
     * @return Eigen::Vector3d ball velocity
     */
    [[nodiscard]] Eigen::Vector3d calculateBallVelocity(const Eigen::Affine2d& old_ball_position,
                                                        const Eigen::Affine2d& new_ball_position,
                                                        const time::TimePoint& old_stamp,
                                                        const time::TimePoint& new_stamp);

    /**
     * @brief Get an 'Eigen::Affine2d' object from 2d coordinates
     *
     * @param x x coordinate
     * @param y y coordinate
     * @return Eigen::Affine2d
     */
    static Eigen::Affine2d getBallAffine(double x, double y);

    /**
     * @brief Set the Initial Vision Values
     *
     * @param data ball positions
     * @param time time of the measurement
     */
    bool setInitialVisionValues(const std::vector<ssl_interface::SSLBallInfo>& data, const time::TimePoint& time);

    /**
     * @brief Get the latest Ball Transform that is valid
     *
     * @return transform::TransformWithVelocity
     */
    [[nodiscard]] transform::TransformWithVelocity getLatestBallTransform() const;

    [[nodiscard]] transform::BallInfo getLatestBallInfo() const;

    [[nodiscard]] bool visionDataInvalid(time::TimePoint time) const;

   private:
    std::string ball_frame;
    std::string global_frame;
    std::vector<std::unique_ptr<BaseFilter>> filters;

    /**
     * @brief latest time point if ball position is known
     *
     */
    std::optional<time::TimePoint> latest_stamp;

    /**
     * @brief latest time point if ball position from vision is known. Only set if approved by the filters
     *
     */
    std::optional<time::TimePoint> latest_vision_stamp;

    /**
     * @brief latest position of the ball if known. Only set if approved by the filters
     *
     */
    std::optional<Eigen::Affine2d> latest_ball_position;

    /**
     * @brief latest position of the ball based on vision if known. Only set if approved by the filters
     *
     */
    std::optional<Eigen::Affine2d> latest_vision_ball_position;

    /**
     * @brief latest velocity of the ball if known. Only set if approved by the filters
     *
     */
    std::optional<Eigen::Vector3d> latest_ball_velocity;

    /**
     * @brief whether the ball is in a robot or not if known
     *
     */
    std::optional<bool> ball_in_robot;

    /**
     * @brief if the @p ball_in_robot is true: the robot that carries the ball
     *
     */
    std::optional<RobotIdentifier> ball_carrying_robot;

    // vector with size_t, bool, time::TimePoint
    std::vector<std::tuple<size_t, bool, time::TimePoint>> camera_status;

    // fourth element is the last time the ball was in the dribbler
    struct BallInDribblerData {
        BallInDribblerData(RobotIdentifier robot_id, bool ball_in_dribbler, time::TimePoint stamp,
                           time::TimePoint last_time_in_dribbler, time::TimePoint last_time_not_in_dribbler)
            : robot_id(robot_id),
              ball_in_dribbler(ball_in_dribbler),
              stamp(stamp)
              //   last_time_in_dribbler(last_time_in_dribbler),
              //   last_time_not_in_dribbler(last_time_not_in_dribbler)
              {};

        RobotIdentifier robot_id;
        bool ball_in_dribbler;
        time::TimePoint stamp;
        // time::TimePoint last_time_in_dribbler;
        // time::TimePoint last_time_not_in_dribbler;
        static constexpr size_t MAX_ENTRIES = 5;
        util::StaticQueue<MAX_ENTRIES, bool> latest_ball_feedback{};
    };
    std::vector<BallInDribblerData> ball_in_feedback_robot;

    std::vector<std::tuple<RobotIdentifier, bool, time::TimePoint>> ball_in_vision_robot;

    std::shared_ptr<transform::WorldModel> world_model;

    // Stores the last average ball velocity vector and the position where that velocity was last evaluated from
    struct VelFilterInfo {
        constexpr static size_t MAX_ENTRIES = 4;
        constexpr static size_t NUM_BALLS_AVERAGED = 3;
        constexpr static double MAX_VEL_TOLERANCE = 8.0;

        // all the past considered velocities (in batches of 3)
        util::StaticQueue<MAX_ENTRIES, Eigen::Vector2d> considered_velocities;

        // the queue of positions that should be batched together
        util::StaticQueue<NUM_BALLS_AVERAGED, std::pair<Eigen::Vector2d, double>> averaged_positions;

        // the last position that resulted in the last batch -> new position compared to this for velocity
        Eigen::Vector2d last_averaged_position{0, 0};

        double last_average_time = 0;
        Eigen::Vector2d avg_vel{0, 0};
    } vel_filter_info;

    // getters
   public:
    [[nodiscard]] const std::string& getBallFrame() const { return ball_frame; }

    [[nodiscard]] const std::string& getGlobalFrame() const { return global_frame; }

    [[nodiscard]] const std::optional<time::TimePoint>& getLatestStamp() const { return latest_stamp; }

    [[nodiscard]] const std::optional<time::TimePoint>& getLatestVisionStamp() const { return latest_vision_stamp; }

    [[nodiscard]] const std::optional<Eigen::Affine2d>& getLatestBallPosition() const { return latest_ball_position; }

    [[nodiscard]] const std::optional<Eigen::Affine2d>& getLatestVisionBallPosition() const {
        return latest_vision_ball_position;
    }

    [[nodiscard]] const std::optional<Eigen::Vector3d>& getLatestBallVelocity() const { return latest_ball_velocity; }

    [[nodiscard]] const std::optional<bool>& getBallInRobot() const { return ball_in_robot; }

    [[nodiscard]] const std::optional<RobotIdentifier>& getBallCarryingRobot() const {
        return ball_carrying_robot;
    }  ////

    bool setFilteredVisionData(const std::vector<ssl_interface::SSLBallInfo>& data, const time::TimePoint& time,
                               size_t camera_id);

    /**
     * @brief updates camera id and accepted status
     *
     */
    void checkCameraStatus(size_t camera_id, bool accepted, time::TimePoint time);

    /**
     * @return true if the ball is invisible
     *
     */
    bool checkBallInvisible();

    /**
     * @brief sets ball_in_vision_robot if a ball is near the dribbler of an enemy robot
     *
     */
    void checkVisionDribbler(Eigen::Affine2d ball_position, time::TimePoint time);

    /**
     * @brief updates ball_in_feedback_robot
     * @param robot_id the robot for which the status should be updated
     * @param has_ball true if the robot has the ball
     * @param time current time
     */
    void setFeedbackDribblerStatus(RobotIdentifier robot_id, bool has_ball, time::TimePoint time);

    /**
     * @brief updates ball_in_vision_robot
     * @param robot_id the robot for which the status should be updated
     * @param has_ball true if the robot has the ball
     * @param time current time
     */
    void setVisionDribblerStatus(RobotIdentifier robot_id, bool has_ball, time::TimePoint time);

    /**
     * @brief returns if the ball is in the dribbler of an ally robot
     * @param time current time
     * @return true if the ball is in the dribbler of an ally robot
     */
    [[nodiscard]] bool getBallInFeedbackDribbler(time::TimePoint time) const;

    /**
     * @brief returns if the ball is in the dribbler of an enemy robot
     * @param time current time
     * @return true if the ball is in the dribbler of an enemy robot
     */
    [[nodiscard]] bool getBallInVisionDribbler(time::TimePoint time) const;

    /**
     * @brief sets the ball_in_robot to false and sets the ball_carrying_robot to std::nullopt
     *        and sets all robots to not have the ball
     */
    [[maybe_unused]] void setBallIsNotInRobot();

    /**
     * @brief returns the robot that has the ball
     * @return the robot that has the ball if it is in a robot or std::nullopt
     */
    [[nodiscard]] std::optional<RobotIdentifier> getRobotWithBall() const;
    time::TimePoint getLatestDribblerFeedbackTime(RobotIdentifier robot_id);
};
}  // namespace luhsoccer::game_data_provider