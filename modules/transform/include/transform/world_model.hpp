/**
 * @file transform.hpp
 * @author Fabrice Zeug (zeug@stud.uni-hannover.de)
 * @brief
 * @version 0.1
 * @date 2022-08-11
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once

#include <Eigen/Geometry>
#include <optional>
#include <utility>
#include <shared_mutex>

// #include "logger.h"
#include "time/time.hpp"
#include "transform/circular_buffer.hpp"
#include "logger/logger.hpp"
#include "robot_identifier.hpp"
#include "transform/game_state.hpp"

namespace luhsoccer::transform {

namespace field {
const std::string CENTER = "field_center";
const std::string MID_LINE_LEFT = "field_mid_line_left";
const std::string MID_LINE_RIGHT = "field_mid_line_right";

const std::string CORNER_ENEMY_LEFT = "field_corner_enemy_left";
const std::string CORNER_ENEMY_RIGHT = "field_corner_enemy_right";
const std::string CORNER_ALLY_LEFT = "field_corner_ally_left";
const std::string CORNER_ALLY_RIGHT = "field_corner_ally_right";

const std::string GOAL_ENEMY_LEFT = "field_goal_enemy_left";
const std::string GOAL_ENEMY_CENTER = "field_goal_enemy_center";
const std::string GOAL_ENEMY_RIGHT = "field_goal_enemy_right";
const std::string GOAL_ALLY_LEFT = "field_goal_ally_left";
const std::string GOAL_ALLY_CENTER = "field_goal_ally_center";
const std::string GOAL_ALLY_RIGHT = "field_goal_ally_right";

const std::string DEFENSE_AREA_CORNER_ENEMY_LEFT = "field_defense_area_corner_enemy_left";
const std::string DEFENSE_AREA_CORNER_ENEMY_RIGHT = "field_defense_area_corner_enemy_right";
const std::string DEFENSE_AREA_CORNER_ALLY_LEFT = "field_defense_area_corner_ally_left";
const std::string DEFENSE_AREA_CORNER_ALLY_RIGHT = "field_defense_area_corner_ally_right";
const std::string DEFENSE_AREA_INTERSECTION_ENEMY_LEFT = "field_defense_area_intersection_enemy_left";
const std::string DEFENSE_AREA_INTERSECTION_ENEMY_RIGHT = "field_defense_area_intersection_enemy_right";
const std::string DEFENSE_AREA_INTERSECTION_ALLY_LEFT = "field_defense_area_intersection_ally_left";
const std::string DEFENSE_AREA_INTERSECTION_ALLY_RIGHT = "field_defense_area_intersection_ally_right";

}  // namespace field

/**
 * @brief Common header for Transform Objects
 *
 */
struct TransformHeader {
    /**
     * @brief frame name of which the transform is described
     *
     */
    std::string child_frame;
    /**
     * @brief frame in which the transform is described
     *
     */
    std::string parent_frame;
    /**
     * @brief point in time of the transform
     *
     */
    time::TimePoint stamp;
};

/**
 * @brief transform between two frames at a given point in time
 *
 */
struct Transform {
    TransformHeader header;
    Eigen::Affine2d transform;
};

/**
 * @brief velocity of child frame relative to parent frame represented in
 * parent frame
 */
struct Velocity {
    TransformHeader header;
    /**
     * @brief frame in which the velocity is represented
     *
     */
    std::string reference_frame;

    /**
     * @brief velocity of the frame (vx, vy, vt) in the parent frames coordinate system
     *
     */
    Eigen::Vector3d velocity;
};

/**
 * @brief Transform and Velocity of child frame in parent frame
 *
 */
struct TransformWithVelocity {
    TransformHeader header;
    /**
     * @brief the transform of the child frame in the parent frames coordinate system
     *
     */
    std::optional<Eigen::Affine2d> transform;
    /**
     * @brief the velocity of the child frame (vx, vy, vt) represented in the parent frames coordinate system
     *
     */
    std::optional<Eigen::Vector3d> velocity;
};
/// default buffer size for transform buffer equals 10s at 1kHz
const size_t DEFAULT_TRANSFORM_BUFFER_SIZE = 10000;

/**
 * @brief Data regarding a robot (ally or enemy)
 *
 */
struct RobotData {
    /**
     * @brief the point in time the data stems from
     *
     */
    time::TimePoint time;
    /**
     * @brief if the robot is on the field
     *
     */
    bool on_field{false};
};
/**
 * @brief Data regarding an ally robot
 *
 */
struct AllyRobotData : public RobotData {
    /**
     * @brief if the ball is held by the dribbler.
     *
     */
    bool ball_in_dribbler{false};
    /**
     * @brief voltage of the kicker capacitors in volts (-1 if unkown).
     *
     */
    int cap_voltage{-1};
};

/**
 * @brief State of the ball
 *
 */
enum class BallState {
    MISSING,   ///< No data of the ball is known
    ON_FIELD,  ///< The ball is on the field but not held by a robot
    IN_ROBOT   ///< The ball is held by a robot
};

/**
 * @brief Data about the playing ball
 *
 */
struct BallInfo {
    /**
     * @brief the point in time the data stems from
     *
     */
    time::TimePoint time;
    /**
     * @brief state of the Ball
     *
     */
    BallState state{BallState::MISSING};
    /**
     * @brief if @p state is BallState::IN_ROBOT, the robot that holds the ball
     *
     */
    std::optional<RobotIdentifier> robot;
    /**
     * @brief the position and velocity of the ball in the field
     *
     */
    std::optional<std::pair<std::optional<Eigen::Affine2d>, std::optional<Eigen::Vector3d>>> position;

    [[nodiscard]] bool isInRobot(const RobotIdentifier& robot) const {
        return this->state == BallState::IN_ROBOT && this->robot.has_value() && this->robot.value() == robot;
    }
};

struct FieldData {
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
    double field_runoff_width{0.3};
};

/**
 * @brief The @struct stores data about all robots (except the position).
 * It is part of the WorldModel and should not be created by yourself or access it.
 *
 */
struct RobotDataStorage {
    template <class T>
    using DataStorage = std::unordered_map<RobotIdentifier, std::shared_ptr<CircularBuffer<T>>>;

    RobotDataStorage()
        : possible_robots(generateAllPossibleRobots(MAX_ROBOTS_PER_TEAM)),
          possible_ally_robots(generatePossibleRobots(MAX_ROBOTS_PER_TEAM, Team::ALLY)),
          possible_enemy_robots(generatePossibleRobots(MAX_ROBOTS_PER_TEAM, Team::ENEMY)) {
        for (size_t i = 0; i < MAX_ROBOTS_PER_TEAM; i++) {
            this->ally_robots.emplace(RobotIdentifier(i, Team::ALLY),
                                      std::make_shared<CircularBuffer<AllyRobotData>>(DEFAULT_TRANSFORM_BUFFER_SIZE));
            this->enemy_robots.emplace(RobotIdentifier(i, Team::ENEMY),
                                       std::make_shared<CircularBuffer<RobotData>>(DEFAULT_TRANSFORM_BUFFER_SIZE));
        }
    };

    RobotDataStorage(const RobotDataStorage& previous)
        : possible_robots(generateAllPossibleRobots(MAX_ROBOTS_PER_TEAM)),
          possible_ally_robots(generatePossibleRobots(MAX_ROBOTS_PER_TEAM, Team::ALLY)),
          possible_enemy_robots(generatePossibleRobots(MAX_ROBOTS_PER_TEAM, Team::ENEMY)) {
        for (size_t i = 0; i < MAX_ROBOTS_PER_TEAM; i++) {
            RobotIdentifier ally(i, Team::ALLY);
            auto previous_ally = previous.ally_robots.find(ally);
            if (previous_ally != previous.ally_robots.end()) {
                this->ally_robots.emplace(ally, std::make_shared<CircularBuffer<AllyRobotData>>(
                                                    DEFAULT_TRANSFORM_BUFFER_SIZE, previous_ally->second));
            } else {
                this->ally_robots.emplace(
                    ally, std::make_shared<CircularBuffer<AllyRobotData>>(DEFAULT_TRANSFORM_BUFFER_SIZE));
            }
            RobotIdentifier enemy(i, Team::ENEMY);
            auto previous_enemy = previous.enemy_robots.find(enemy);
            if (previous_enemy != previous.enemy_robots.end()) {
                this->enemy_robots.emplace(enemy, std::make_shared<CircularBuffer<RobotData>>(
                                                      DEFAULT_TRANSFORM_BUFFER_SIZE, previous_enemy->second));
            } else {
                this->enemy_robots.emplace(enemy,
                                           std::make_shared<CircularBuffer<RobotData>>(DEFAULT_TRANSFORM_BUFFER_SIZE));
            }
        }
    };

    RobotDataStorage& operator=(const RobotDataStorage&) = delete;
    RobotDataStorage(const RobotDataStorage&&) = delete;
    RobotDataStorage& operator=(const RobotDataStorage&&) = delete;
    ~RobotDataStorage() = default;

    DataStorage<AllyRobotData> ally_robots;
    DataStorage<RobotData> enemy_robots;

    const std::vector<RobotIdentifier> possible_robots;
    const std::vector<RobotIdentifier> possible_ally_robots;
    const std::vector<RobotIdentifier> possible_enemy_robots;

    /**
     * @brief Represents an empty handle which is never equal to a real robot. Every function which accepts an handle
     * must not throw an exception will an empty handle is given as an argument.
     */
    constexpr inline static RobotIdentifier EMPTY_HANDLE{std::numeric_limits<unsigned int>().max(), Team::ALLY};

    const static std::vector<RobotIdentifier> generatePossibleRobots(size_t robot_num, Team team) {
        std::vector<RobotIdentifier> v;
        for (size_t i = 0; i < robot_num; i++) {
            // NOLINTNEXTLINE(modernize-use-emplace) - vector cant create RobotIdentifier
            v.push_back(RobotIdentifier(i, team));
        }
        return v;
    };
    const static std::vector<RobotIdentifier> generateAllPossibleRobots(size_t robot_num) {
        std::vector<RobotIdentifier> v;
        for (size_t i = 0; i < robot_num; i++) {
            // NOLINTNEXTLINE(modernize-use-emplace) - vector cant create RobotIdentifier
            v.push_back(RobotIdentifier(i, Team::ALLY));
            // NOLINTNEXTLINE(modernize-use-emplace) - vector cant create RobotIdentifier
            v.push_back(RobotIdentifier(i, Team::ENEMY));
        }
        return v;
    };
};
/**
 * @brief The default displacement of a ball when it is in the dribbler, measured from the center in x direction of a
 * robot in m
 *
 */
constexpr double DEFAULT_BALL_DISPLACEMENT = 0.09;
/**
 * @brief The default pose if a robot is not on the field
 *
 */
const Eigen::Affine2d OUT_OF_GAME_TRANSFORM = Eigen::Translation2d(100, 0) * Eigen::Rotation2Dd(0);

/**
 * @brief Class that stores and manages a model of the world with transforms
 * @note objects of this class can be used in multiple threads as every member function is threadsafe.
 * @note WorldModel objects are usually stored in std::shared_ptr.
 */
class WorldModel {
   public:
    /**
     * @brief Construct a new Main World Model object
     *
     * @param global_frame frame name of the global parent frame
     * @param ball_frame frame name of the ball frame
     * @param ball_displacement distance between robot center and ball center when the ball is in the dribbler
     */

    WorldModel(std::string global_frame = "world", std::string ball_frame = "ball",
               double ball_displacement = DEFAULT_BALL_DISPLACEMENT);
    ~WorldModel() = default;

    WorldModel(WorldModel&&) = delete;
    WorldModel& operator=(WorldModel&&) = delete;
    WorldModel& operator=(const WorldModel&) = delete;

    /**
     * @brief Construct a new alternative World Model object that.
     *
     * @param previous the previous (main) world model
     * @warning instead of copying the data, the a reference on the previous @class is stored
     */
    WorldModel(const WorldModel& previous);

    /**
     * @brief Get the a the transform of child_frame in @p parent_frame 's coordinate system at given time point
     *
     * @param child_frame frame of which the transform is requested
     * @param parent_frame frame in which the transform is requested, leave empty for global frame
     * @param time point in time of transform, time::TimePoint(0) means latest data
     * @return std::optional<Transform> requested transform if possible
     */
    [[nodiscard]] std::optional<Transform> getTransform(const std::string& child_frame,
                                                        const std::string& parent_frame = "",
                                                        time::TimePoint time = time::TimePoint(0),
                                                        bool lock = true) const;

    static constexpr double DEFAULT_AVERAGING_INTERVAL = 1.0 / 60.0;
    /**
     * @brief Get the velocity of child_frame relative to parent_frame represented
     * in reference_frame -> observation frame
     *
     * @param child_frame frame of which the transform is requested
     * @param parent_frame frame relative to which the velocity is requested, leave empty for global frame
     * @param reference_frame frame in which the velocity is given, leave empty for global frame
     * @param time point in time of transform, time::TimePoint(0) means latest data
     * @param averaging_interval period of time to average the velocity if no velocity is provided
     * @return std::optional<Velocity> requested velocity in m/s if possible
     */
    [[nodiscard]] std::optional<Velocity> getVelocity(
        const std::string& child_frame, const std::string& parent_frame = "", const std::string& reference_frame = "",
        time::TimePoint time = time::TimePoint(0),
        time::Duration averaging_interval = time::Duration(DEFAULT_AVERAGING_INTERVAL), bool lock = true) const;

    /// return a list of all registered frames
    [[nodiscard]] std::vector<std::string> getAllTransformFrames() const;

    /// return the name of the global frame
    [[nodiscard]] inline std::string getGlobalFrame() const { return this->global_frame; };

    /// return the name of the ball frame
    [[nodiscard]] inline std::string getBallFrame() const { return this->ball_frame; };

    /**
     * @brief push new data to the buffer
     *
     * @param transform new transform of the frame
     * @param static_frame whether the frame is static
     * @return if pushing was successful
     */
    bool pushTransform(TransformWithVelocity transform, bool static_frame = false);

    // --------------------------------Robots-----------------------------------
    /**
     * @brief Get the data of an ally robot
     *
     * @param identifier ally robot
     * @param time point in time when of which the data is requested, time::TimePoint(0) means latest data
     * @return std::optional<AllyRobotData> the latest available data regarding the given robot at the given point in
     * time
     */
    [[nodiscard]] std::optional<AllyRobotData> getAllyRobotData(const RobotIdentifier& identifier,
                                                                const time::TimePoint& time = time::TimePoint(0)) const;

    [[nodiscard]] std::optional<RobotData> getRobotData(const RobotIdentifier& identifier,
                                                        const time::TimePoint& time = time::TimePoint(0)) const;

    bool pushEnemyRobotData(const RobotIdentifier& identifier, const RobotData& data);

    bool pushAllyRobotData(const RobotIdentifier& identifier, const AllyRobotData& data);

    bool removeRobotFromField(const RobotIdentifier& identifier);

    /**
     * @brief Returns a vector of the possible robots on the field
     * All the handles could be invalid, since there is no guarantee, that the robot are actually on the field
     * @return std::vector<RobotHandle>
     */
    [[nodiscard]] std::vector<RobotIdentifier> getPossibleRobots() const;

    /**
     * @brief Returns a vector of the possible robots on the field for a specific team
     * All the handles could be invalid, since there is no guarantee, that the robot are actually on the field
     * @tparam The team
     * @return std::vector<RobotHandle>
     */
    template <enum Team>
    [[nodiscard]] std::vector<RobotIdentifier> getPossibleRobots() const;

    /**
     * @brief Returns a vector of the currently visible robots on the field
     * The handles may be used for further identification of the robots but the handles could get invalid in the mean
     * time. You could check this by using the isHandleStillValid function
     * @return std::vector<RobotHandle>
     */
    [[nodiscard]] std::vector<RobotIdentifier> getVisibleRobots(const time::TimePoint& time = time::TimePoint(0)) const;

    /**
     * @brief Get the visible robots for a specific team
     * The handles may be used for further identification of the robots but the handles could get invalid in the mean
     * time. You could check this by using the isHandleStillValid function
     *
     * @tparam The team
     * @return std::vector<RobotHandle>
     */
    template <enum Team>
    [[nodiscard]] std::vector<RobotIdentifier> getVisibleRobots(const time::TimePoint& time = time::TimePoint(0)) const;

    // ---------------------------Ball------------------------------------------
    bool pushNewBallInfo(const BallInfo& ball_info);

    bool updateBallPosition(const BallInfo& ball_info);

    std::optional<BallInfo> getBallInfo(const time::TimePoint& time = time::TimePoint(0)) const;

    void setLastBallObtainPosition(const time::TimePoint& time, const Eigen::Affine2d& position) {
        const std::unique_lock<std::shared_mutex> lock(this->ball_obtain_mutex);
        this->last_ball_obtain_position = {time, position};
    }

    void removeLastBallObtainPosition() {
        const std::unique_lock<std::shared_mutex> lock(this->ball_obtain_mutex);
        this->last_ball_obtain_position = std::nullopt;
    }

    std::optional<std::pair<time::TimePoint, Eigen::Affine2d>> getLastBallObtainPosition() const {
        const std::shared_lock<std::shared_mutex> lock(this->ball_obtain_mutex);
        return this->last_ball_obtain_position;
    }
    // ---------------------------GameState-------------------------------------
    bool pushNewGameState(const GameState& game_state, const time::TimePoint& time);

    std::optional<GameState> getGameState(const time::TimePoint& time = time::TimePoint(0)) const;

    // ---------------------------FieldData-------------------------------------
    void setFieldData(const FieldData& field_data) {
        std::unique_lock lock(this->field_data_mtx);
        this->field_data = field_data;
    }

    FieldData getFieldData() const {
        std::shared_lock lock(this->field_data_mtx);
        return this->field_data;
    }

    // ---------------------------FieldData-------------------------------------
    void setGoalieId(const RobotIdentifier& goalie_id) {
        std::unique_lock lock(this->goalie_id_mtx);
        this->goalie_id = goalie_id;
    }

    std::optional<RobotIdentifier> getGoalieID() const {
        std::shared_lock lock(this->goalie_id_mtx);
        return this->goalie_id;
    };

   private:
    /// Transform of a frame at given point in time
    struct FrameTransformElement {
        std::optional<Eigen::Affine2d> transform;
        std::optional<Eigen::Vector3d> velocity;
        time::TimePoint stamp;
    };

    /// Transform of a frame for all time points
    struct FrameTransform {
        FrameTransform() : buffer(new CircularBuffer<FrameTransformElement>(DEFAULT_TRANSFORM_BUFFER_SIZE)){};
        FrameTransform(const FrameTransform& previous)
            : child_frame(previous.child_frame),
              buffer(new CircularBuffer<FrameTransformElement>(DEFAULT_TRANSFORM_BUFFER_SIZE, previous.buffer)),
              static_frame(previous.static_frame){};

        FrameTransform& operator=(const FrameTransform&) = delete;
        FrameTransform(const FrameTransform&&) = delete;
        FrameTransform& operator=(const FrameTransform&&) = delete;
        ~FrameTransform() = default;

        std::string child_frame;  // NOLINT(misc-non-private-member-variables-in-classes) - is struct
        std::shared_ptr<CircularBuffer<FrameTransformElement>>
            buffer;           // NOLINT(misc-non-private-member-variables-in-classes) - is struct
        bool static_frame{};  // NOLINT(misc-non-private-member-variables-in-classes) - is struct
    };

    /// return the Frame Transform Element before and after a given time point
    std::optional<std::array<FrameTransformElement, 2>> getTransformElementBeforeAndAfterTimePoint(
        const FrameTransform& child_frame_transform, time::TimePoint& time) const;

    /// name of the global frame
    const std::string global_frame;
    const std::string ball_frame;
    const double ball_displacement;
    const Eigen::Affine2d ball_displacement_transform;

    mutable std::shared_mutex transform_storage_mutex;
    /// actual buffer storage
    std::unordered_map<std::string, FrameTransform> transform_storage;

    RobotDataStorage robot_data;

    std::shared_ptr<CircularBuffer<std::pair<time::TimePoint, GameState>>> game_states;

    std::shared_ptr<CircularBuffer<BallInfo>> ball_infos;

    std::optional<std::pair<time::TimePoint, Eigen::Affine2d>> last_ball_obtain_position;
    mutable std::shared_mutex ball_obtain_mutex;

    mutable std::shared_mutex field_data_mtx;
    FieldData field_data;

    mutable std::shared_mutex goalie_id_mtx;
    std::optional<RobotIdentifier> goalie_id;

    /// Logger for prints
    logger::Logger logger;
};

}  // namespace luhsoccer::transform
