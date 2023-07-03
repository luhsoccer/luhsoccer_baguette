#pragma once

#include <optional>
#include <Eigen/Geometry>
#include <time/time.hpp>
#include "common_types.hpp"
namespace luhsoccer::ssl_interface {
/**
 * @brief A struct that wraps the ssl vision data about a single robot
 *
 */
struct SSLRobotInfo {
    /// The id of the robot
    unsigned int id{0};

    /// The transform of the robot
    Eigen::Affine2d transform{};

    /// A value between 0.0 - 1.0 which show how accurate the vision can detect this robot
    double confidence{0.0};

    /// On which pixels of the camera the robot was detected
    Eigen::Vector2d pixel_position;
};

/// Wraps the ssl ball data
struct SSLBallInfo {
    /// The position of the ball
    Eigen::Vector3d position{0.0, 0.0, 0.0};

    /// A value between 0.0 - 1.0 which show how accurate the vision can detect this ball
    double confidence{0.0};

    /// On which pixels of the camera the ball was detected
    Eigen::Vector2d pixel_position;
};

/// @brief The possible line types
/// Each line from the game field has a distinct type. This enum contains all possible values.
enum class SSLLineType {
    TOP_TOUCH_LINE,
    BOTTOM_TOUCH_LINE,
    LEFT_GOAL_LINE,
    RIGHT_GOAL_LINE,
    HALFWAY_LINE,
    CENTER_LINE,
    LEFT_PENALTY_STRETCH,
    RIGHT_PENALTY_STRETCH,
    LEFT_FIELD_LEFT_PENALTY_STRETCH,
    LEFT_FIELD_RIGHT_PENALTY_STRETCH,
    RIGHT_FIELD_LEFT_PENALTY_STRETCH,
    RIGHT_FIELD_RIGHT_PENALTY_STRETCH,
    UNKNOWN
};

/// A single line object
struct SSLFieldLine {
    std::string name;
    /// The type of the line
    SSLLineType type{SSLLineType::UNKNOWN};

    /// The first point of the line.
    Eigen::Vector2d start_point{0.0, 0.0};

    /// The last point of the line.
    Eigen::Vector2d end_point{0.0, 0.0};

    /// The thickness of the line.
    double thickness{0.0};
};

/// Each arc type that exists on the game field.
enum class SSLArcType { CENTER_CIRCLE, UNKNOWN };

/// A single arc in the game field
struct SSLFieldArc {
    std::string name;
    /// The type of the arc
    SSLArcType type{SSLArcType::UNKNOWN};

    /// The center pos of the arc.
    Eigen::Vector2d center{0.0, 0.0};

    /// The start angle from the arc in radians.
    double start_angle{0.0};

    /// The end angle from the arc in radians.
    double end_angle{0.0};

    /// The radius of the arc in meters.
    double radius{0.0};

    /// The thickness of the arc line in meters.
    double thickness{0.0};
};

///  The data which represent the game field
struct SSLFieldData {
    /// The size of the game field in meters
    /// X represent the length of field and Y the width.

    Division division;
    Eigen::Vector2d size{0.0, 0.0};

    double goal_width{0.0};
    double goal_depth{0.0};

    double boundary_width{0.0};

    double penalty_area_depth{0.0};
    double penalty_area_width{0.0};
    double center_circle_radius{0.0};
    double line_thickness{0.0};
    double goal_center_to_penalty_mark{0.0};
    double goal_height{0.0};
    double ball_radius{0.0};
    double max_robot_radius{0.0};

    /// All lines on the game field
    std::vector<SSLFieldLine> lines{};

    /// All arc on the game field
    std::vector<SSLFieldArc> arcs{};

    // field
    Eigen::Vector2d field_left_top{0.0, 0.0};
    Eigen::Vector2d field_left_bottom{0.0, 0.0};
    Eigen::Vector2d field_right_top{0.0, 0.0};
    Eigen::Vector2d field_right_bottom{0.0, 0.0};
    Eigen::Vector2d field_left_center{0.0, 0.0};
    Eigen::Vector2d field_right_center{0.0, 0.0};
    Eigen::Vector2d field_center{0.0, 0.0};
    Eigen::Vector2d field_top_center{0.0, 0.0};
    Eigen::Vector2d field_bottom_center{0.0, 0.0};

    // penalty area
    Eigen::Vector2d penalty_area_left_baseline_top{0.0, 0.0};
    Eigen::Vector2d penalty_area_right_baseline_top{0.0, 0.0};
    Eigen::Vector2d penalty_area_left_baseline_bottom{0.0, 0.0};
    Eigen::Vector2d penalty_area_right_baseline_bottom{0.0, 0.0};
    Eigen::Vector2d penalty_area_left_field_top{0.0, 0.0};
    Eigen::Vector2d penalty_area_right_field_top{0.0, 0.0};
    Eigen::Vector2d penalty_area_left_field_bottom{0.0, 0.0};
    Eigen::Vector2d penalty_area_right_field_bottom{0.0, 0.0};
    Eigen::Vector2d penalty_area_left_field_center{0.0, 0.0};
    Eigen::Vector2d penalty_area_right_field_center{0.0, 0.0};

    // goal
    Eigen::Vector2d goal_left_top{0.0, 0.0};
    Eigen::Vector2d goal_left_bottom{0.0, 0.0};
    Eigen::Vector2d goal_right_top{0.0, 0.0};
    Eigen::Vector2d goal_right_bottom{0.0, 0.0};
};

/// The data from the vision
struct SSLVisionData {
    /// All blue robots that were found by the vision
    std::vector<SSLRobotInfo> blue_robots{};

    /// All yellow robots that were found by the vision
    std::vector<SSLRobotInfo> yellow_robots{};

    /// All balls that were found by the vision
    std::vector<SSLBallInfo> balls{};

    /// A timestamp in seconds from the vision that represents the capture timepoint
    time::TimePoint timestamp_capture;

    /// A timestamp in seconds from the vision that represents the send timepoint
    time::TimePoint timestamp_sent;

    /// The frame number
    size_t frame_number;

    /// The camera from which the frame was send
    size_t camera_id;
};

/**
 * @brief All the data in one wrapper packet
 *
 */
struct SSLWrapperData {
    std::optional<SSLVisionData> vision;
    std::optional<SSLFieldData> field;
    // TODO maybe add geometry data
};

enum class SSLCommandType {
    // All robots should completely stop moving.
    HALT = 0,
    // Robots must keep 50 cm from the ball.
    STOP = 1,
    // A prepared kickoff or penalty may now be taken.
    NORMAL_START = 2,
    // The ball is dropped and free for either team.
    FORCE_START = 3,
    // The yellow team may move into kickoff position.
    PREPARE_KICKOFF_YELLOW = 4,
    // The blue team may move into kickoff position.
    PREPARE_KICKOFF_BLUE = 5,
    // The yellow team may move into penalty position.
    PREPARE_PENALTY_YELLOW = 6,
    // The blue team may move into penalty position.
    PREPARE_PENALTY_BLUE = 7,
    // The yellow team may take a direct free kick.
    DIRECT_FREE_YELLOW = 8,
    // The blue team may take a direct free kick.
    DIRECT_FREE_BLUE = 9,
    // The yellow team may take an indirect free kick.
    INDIRECT_FREE_YELLOW = 10,
    // The blue team may take an indirect free kick.
    INDIRECT_FREE_BLUE = 11,
    // The yellow team is currently in a timeout.
    TIMEOUT_YELLOW = 12,
    // The blue team is currently in a timeout.
    TIMEOUT_BLUE = 13,
    // Possible goal
    GOAL_YELLOW = 14,
    // Possible goal
    GOAL_BLUE = 15,
    // Equivalent to STOP, but the yellow team must pick up the ball and
    // drop it in the Designated Position.
    BALL_PLACEMENT_YELLOW = 16,
    // Equivalent to STOP, but the blue team must pick up the ball and drop
    // it in the Designated Position.
    BALL_PLACEMENT_BLUE = 17
};

enum class SSLGameEventType {
    UNKNOWN = 0,
    BALL_LEFT_FIELD_TOUCH_LINE = 6,
    BALL_LEFT_FIELD_GOAL_LINE = 7,
    AIMLESS_KICK = 11,
    ATTACKER_TOO_CLOSE_TO_DEFENSE_AREA = 19,
    DEFENDER_IN_DEFENSE_AREA = 31,
    BOUNDARY_CROSSING = 41,
    KEEPER_HELD_BALL = 13,
    BOT_DRIBBLED_BALL_TOO_FAR = 17,
    BOT_PUSHED_BOT = 24,
    BOT_HELD_BALL_DELIBERATELY = 26,
    BOT_TIPPED_OVER = 27,
    ATTACKER_TOUCHED_BALL_IN_DEFENSE_AREA = 15,
    BOT_KICKED_BALL_TOO_FAST = 18,
    BOT_CRASH_UNIQUE = 22,
    BOT_CRASH_DRAWN = 21,
    DEFENDER_TOO_CLOSE_TO_KICK_POINT = 29,
    BOT_TOO_FAST_IN_STOP = 28,
    BOT_INTERFERED_PLACEMENT = 20,
    POSSIBLE_GOAL = 39,
    GOAL = 8,
    INVALID_GOAL = 42,
    ATTACKER_DOUBLE_TOUCHED_BALL = 14,
    PLACEMENT_SUCCEEDED = 5,
    PENALTY_KICK_FAILED = 43,
    NO_PROGRESS_IN_GAME = 2,
    PLACEMENT_FAILED = 3,
    MULTIPLE_CARDS = 32,
    MULTIPLE_FOULS = 34,
    BOT_SUBSTITUTION = 37,
    TOO_MANY_ROBOTS = 38,
    CHALLENGE_FLAG = 44,
    EMERGENCY_STOP = 45,
    UNSPORTING_BEHAVIOR_MINOR = 35,
    UNSPORTING_BEHAVIOR_MAJOR = 36,

    // Added as an alias to every deprecated event
    DEPRECATED = 255
};

/**
 * @brief A struct representing the team info
 *
 */
struct SSLTeamInfo {
    std::string name{"UNKNOWN"};
    unsigned int score{0};
    unsigned int red_cards{0};
    unsigned int yellow_cards{0};
    std::vector<time::TimePoint> yellow_card_times{};
    unsigned int timeouts{0};
    time::TimePoint timeout_time{};
    unsigned int goalkeeper{0};
    std::optional<unsigned int> foul_counter;
    std::optional<unsigned int> ball_placement_failures;
    std::optional<bool> can_place_ball;
    std::optional<unsigned int> max_allowed_bots;
    std::optional<bool> bot_substitution_intent;
    std::optional<bool> ball_placement_failures_reached;
};

/**
 * @brief The data from the game controller
 *
 */
struct SSLGameControllerData {
    /**
     * @brief The current command that was issued from the game controller
     *
     */
    SSLCommandType command{SSLCommandType::HALT};

    /**
     * @brief The next command, if any
     *
     */
    std::optional<SSLCommandType> next_command;

    /**
     * @brief The team info of the blue team
     *
     */
    SSLTeamInfo blue_team_info;

    /**
     * @brief The team info of the yellow team
     *
     */
    SSLTeamInfo yellow_team_info;

    /**
     * @brief A timestamp in seconds from the gc that represents the issues timepoint of the command
     *
     */
    time::TimePoint timestamp_issued;
    /**
     * @brief A timestamp in seconds from the gc that represents the send timepoint
     *
     */
    time::TimePoint timestamp_sent;

    /**
     * @brief A designated position. May be used for ball placement
     *
     */
    std::optional<Eigen::Vector2d> designated_position;
};
}  // namespace luhsoccer::ssl_interface