#include <utility>

#include "transform/world_model.hpp"

namespace luhsoccer::transform {

bool operator==(const FieldData& lhs, const FieldData& rhs) {
    constexpr double EPS = 0.001;

    // clang-format off
    return lhs.division == rhs.division &&
           std::abs(lhs.size.x() - rhs.size.x()) < EPS &&
           std::abs(lhs.size.y() - rhs.size.y()) < EPS &&
           std::abs(lhs.goal_width - rhs.goal_width) < EPS &&
           std::abs(lhs.goal_depth - rhs.goal_depth) < EPS &&
           std::abs(lhs.boundary_width - rhs.boundary_width) < EPS &&
           std::abs(lhs.penalty_area_depth - rhs.penalty_area_depth) < EPS &&
           std::abs(lhs.penalty_area_width - rhs.penalty_area_width) < EPS &&
           std::abs(lhs.center_circle_radius - rhs.center_circle_radius) < EPS &&
           std::abs(lhs.line_thickness - rhs.line_thickness) < EPS &&
           std::abs(lhs.goal_center_to_penalty_mark - rhs.goal_center_to_penalty_mark) < EPS &&
           std::abs(lhs.goal_height - rhs.goal_height) < EPS &&
           std::abs(lhs.ball_radius - rhs.ball_radius) < EPS &&
           std::abs(lhs.max_robot_radius - rhs.max_robot_radius) < EPS &&
           std::abs(lhs.field_runoff_width - rhs.field_runoff_width) < EPS;
    // clang-format on
}

RobotDataStorage::RobotDataStorage()
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

RobotDataStorage::RobotDataStorage(const RobotDataStorage& previous)
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
            this->ally_robots.emplace(ally,
                                      std::make_shared<CircularBuffer<AllyRobotData>>(DEFAULT_TRANSFORM_BUFFER_SIZE));
        }
        RobotIdentifier enemy(i, Team::ENEMY);
        auto previous_enemy = previous.enemy_robots.find(enemy);
        if (previous_enemy != previous.enemy_robots.end()) {
            this->enemy_robots.emplace(enemy, std::make_shared<CircularBuffer<RobotData>>(DEFAULT_TRANSFORM_BUFFER_SIZE,
                                                                                          previous_enemy->second));
        } else {
            this->enemy_robots.emplace(enemy,
                                       std::make_shared<CircularBuffer<RobotData>>(DEFAULT_TRANSFORM_BUFFER_SIZE));
        }
    }
};

const std::vector<RobotIdentifier> RobotDataStorage::generatePossibleRobots(size_t robot_num, Team team) {
    std::vector<RobotIdentifier> v;
    for (size_t i = 0; i < robot_num; i++) {
        // NOLINTNEXTLINE(modernize-use-emplace) - vector cant create RobotIdentifier
        v.push_back(RobotIdentifier(i, team));
    }
    return v;
};

const std::vector<RobotIdentifier> RobotDataStorage::generateAllPossibleRobots(size_t robot_num) {
    std::vector<RobotIdentifier> v;
    for (size_t i = 0; i < robot_num; i++) {
        // NOLINTNEXTLINE(modernize-use-emplace) - vector cant create RobotIdentifier
        v.push_back(RobotIdentifier(i, Team::ALLY));
        // NOLINTNEXTLINE(modernize-use-emplace) - vector cant create RobotIdentifier
        v.push_back(RobotIdentifier(i, Team::ENEMY));
    }
    return v;
};

WorldModel::WorldModel(std::string global_frame, std::string ball_frame, double ball_displacement)
    : global_frame(std::move(global_frame)),
      ball_frame(std::move(ball_frame)),
      ball_displacement(ball_displacement),
      ball_displacement_transform(Eigen::Translation2d(ball_displacement, 0) * Eigen::Rotation2Dd(0)),
      game_states(new CircularBuffer<std::pair<time::TimePoint, GameState>>(DEFAULT_TRANSFORM_BUFFER_SIZE)),
      ball_infos(new CircularBuffer<BallInfo>(DEFAULT_TRANSFORM_BUFFER_SIZE)),
      logger("transform::WorldModel") {
    // init global frame
    const std::unique_lock lock(this->transform_storage_mutex);
    auto& global_frame_transform = this->transform_storage.try_emplace(this->global_frame).first->second;
    global_frame_transform.child_frame = this->global_frame;
    global_frame_transform.static_frame = true;

    FrameTransformElement zero_transform;
    zero_transform.stamp = time::TimePoint(0.0);
    zero_transform.transform = Eigen::Translation2d(0, 0) * Eigen::Rotation2Dd(0);
    zero_transform.velocity = Eigen::Vector3d::Zero();
    global_frame_transform.buffer->push(zero_transform);
}

WorldModel::WorldModel(const WorldModel& previous)
    : global_frame(previous.global_frame),
      ball_frame(previous.ball_frame),
      ball_displacement(previous.ball_displacement),
      ball_displacement_transform(Eigen::Translation2d(previous.ball_displacement, 0) * Eigen::Rotation2Dd(0)),
      robot_data(previous.robot_data),
      game_states(new CircularBuffer<std::pair<time::TimePoint, GameState>>(DEFAULT_TRANSFORM_BUFFER_SIZE,
                                                                            previous.game_states)),
      ball_infos(new CircularBuffer<BallInfo>(DEFAULT_TRANSFORM_BUFFER_SIZE, previous.ball_infos)),
      last_ball_obtain_position(previous.last_ball_obtain_position),
      field_data(previous.getFieldData()),
      goalie_id(previous.getGoalieID()),
      logger("transform::WorldModel") {
    const std::unique_lock lock(this->transform_storage_mutex);
    for (const auto& frame : previous.transform_storage) {
        this->transform_storage.insert({frame.first, frame.second});
    }
}

std::vector<std::string> WorldModel::getAllTransformFrames() const {
    std::vector<std::string> frames;
    std::shared_lock lock(this->transform_storage_mutex);
    frames.reserve(this->transform_storage.size());
    for (auto& frame_element : this->transform_storage) {
        frames.push_back(frame_element.first);
    }
    return frames;
}

std::optional<std::array<WorldModel::FrameTransformElement, 2>> WorldModel::getTransformElementBeforeAndAfterTimePoint(
    const FrameTransform& child_frame_transform, time::TimePoint& time) const {
    std::array<FrameTransformElement, 2> transform_elements;
    FrameTransformElement& transform_before = transform_elements[0];
    FrameTransformElement& transform_after = transform_elements[1];
    if (child_frame_transform.buffer->size() == 0) {
        this->logger.warning("No data received yet. Buffer empty!");
        return std::nullopt;
    }
    try {
        if (child_frame_transform.static_frame) {
            transform_before = child_frame_transform.buffer->at(0);
            transform_after = child_frame_transform.buffer->at(0);
            time = transform_after.stamp;
        } else if (time.asSec() == 0.0 &&
                   child_frame_transform.buffer->size() > 1) {  // check for time(0) to get latest frame
            transform_before = child_frame_transform.buffer->at(1);
            transform_after = child_frame_transform.buffer->at(0);
            time = transform_after.stamp;

        } else {
            bool found = false;
            // find transform before and after the frame
            for (size_t i = 0; i < child_frame_transform.buffer->size() - 1; i++) {
                if (child_frame_transform.buffer->at(i).stamp <= time) {
                    transform_before = child_frame_transform.buffer->at(i);
                    if (i == 0) {
                        transform_before = child_frame_transform.buffer->at(i + 1);
                        transform_after = child_frame_transform.buffer->at(i);
                    } else {
                        transform_before = child_frame_transform.buffer->at(i);
                        transform_after = child_frame_transform.buffer->at(i - 1);
                    }
                    found = true;
                    break;
                }
            }
            if (!found && child_frame_transform.buffer->size() > 1) {
                this->logger.warning(
                    "TimePoint {} for frame {} is too far in the past. Buffer not big enough! This can lead to huge "
                    "performance "
                    "problems. Check as soon as possible!",
                    time, child_frame_transform.child_frame);
                return std::nullopt;
            }
        }

    } catch (std::out_of_range& e) {
        this->logger.warning("Error {} caught. No data received yet. Buffer empty!", e.what());
        return std::nullopt;
    }
    return transform_elements;
}

bool WorldModel::pushTransform(TransformWithVelocity transform, bool static_frame) {
    // check if frame is empty so global frame
    if (transform.header.parent_frame.empty()) transform.header.parent_frame = this->global_frame;

    std::shared_lock read_lock(this->transform_storage_mutex);
    auto iter = this->transform_storage.find(transform.header.child_frame);

    if (iter == this->transform_storage.end()) {
        read_lock.unlock();
        std::unique_lock write_lock(this->transform_storage_mutex);

        //  child frame not known. Create new frame transform
        iter = this->transform_storage.try_emplace(transform.header.child_frame).first;
        auto& new_transform = iter->second;
        new_transform.static_frame = static_frame;
        new_transform.child_frame = transform.header.child_frame;
        write_lock.unlock();
        read_lock.lock();
    }

    auto& child_frame_transform = iter->second;
    if (child_frame_transform.buffer->size() != 0 &&
        child_frame_transform.buffer->at(0).stamp >= transform.header.stamp && !static_frame) {
        this->logger.warning(
            "Data in Buffer of frame \"{}\" is newer! Newest data of Buffer is from {}. Trying to push data for {}",
            transform.header.child_frame, child_frame_transform.buffer->at(0).stamp, transform.header.stamp);
        return false;
    }

    // transform info global frame and add transform
    std::optional<Transform> parent_transform =
        getTransform(transform.header.parent_frame, this->global_frame, time::TimePoint(0));
    if (!parent_transform) {
        this->logger.warning("Parent frame \"{}\" not known! Cant push transform!", transform.header.parent_frame);
        return false;
    }
    FrameTransformElement transform_in_global_frame;
    transform_in_global_frame.stamp = transform.header.stamp;

    // add transform if provided
    if (transform.transform) {
        transform_in_global_frame.transform = parent_transform->transform.matrix() * transform.transform->matrix();
        // add velocity if provided
        if (transform.velocity) {
            transform_in_global_frame.velocity = transform.velocity;
            transform_in_global_frame.velocity->head(2) =
                Eigen::Rotation2Dd(parent_transform->transform.rotation()).toRotationMatrix() *
                transform_in_global_frame.velocity->head(2);
        }
    }
    child_frame_transform.buffer->push(transform_in_global_frame);
    //    child_frame_transform.static_frame = static_frame;
    return true;
}
// NOLINTNEXTLINE(misc-no-recursion) - recursion gets cut when frames are the same
std::optional<Transform> WorldModel::getTransform(const std::string& child, const std::string& parent,
                                                  time::TimePoint time, bool lock) const {
    // check for same frame
    if (child == parent) {
        Transform transform;
        transform.header.child_frame = child;
        transform.header.parent_frame = parent;
        transform.header.stamp = time;
        transform.transform = Eigen::Translation2d(0, 0) * Eigen::Rotation2Dd(0);
        return transform;
    }

    // check if frame is empty so global frame
    // if (child_frame.empty()) child_frame = this->global_frame;
    const std::string& child_frame = child.empty() ? this->global_frame : child;
    // check if frame is empty so global frame
    const std::string& parent_frame = parent.empty() ? this->global_frame : parent;

    std::shared_lock read_lock(this->transform_storage_mutex, std::defer_lock);
    if (lock) {
        read_lock.lock();
    }

    if (!this->transform_storage.contains(parent_frame)) {
        return std::nullopt;
    }
    auto child_iter = this->transform_storage.find(child_frame);
    if (child_iter == this->transform_storage.end()) {
        return std::nullopt;
    }

    std::optional<Transform> parent_transform = getTransform(parent_frame, this->global_frame, time, false);
    // returns zero transform if parent frame is global frame because of same
    // frame check
    if (!parent_transform) {
        return std::nullopt;
    }
    // FrameTransformElement transform_before, transform_after;
    auto& child_frame_transform = child_iter->second;

    auto transforms = getTransformElementBeforeAndAfterTimePoint(child_frame_transform, time);
    if (!transforms) return std::nullopt;

    auto [transform_before, transform_after] = *transforms;

    // linear extra-/interpolation
    if (transform_before.transform && transform_after.transform) {
        Transform transform;
        transform.header.child_frame = child_frame;
        transform.header.parent_frame = parent_frame;
        transform.header.stamp = time;

        // if extrapolation -> dt > 1.0 else 0.0 < dt < 1.0;
        double dt = 0.0;
        if ((transform_after.stamp - transform_before.stamp).count() != 0) {
            // NOLINTNEXTLINE(bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
            dt = (time - transform_before.stamp).count() /
                 (double)(transform_after.stamp - transform_before.stamp).count();
        } else {
            dt = 0.0;
        }
        Eigen::Translation2d translation(
            transform_before.transform->translation() +
            (transform_after.transform->translation() - transform_before.transform->translation()) * dt);
        Eigen::Rotation2Dd rotation_before(transform_before.transform->rotation());
        Eigen::Rotation2Dd rotation_after(transform_after.transform->rotation());
        double rot_diff = rotation_after.angle() - rotation_before.angle();
        if (rot_diff > L_PI) {
            rot_diff -= 2 * L_PI;
        } else if (rot_diff < -L_PI) {
            rot_diff += 2 * L_PI;
        }
        Eigen::Rotation2Dd rotation(rotation_before.angle() + rot_diff * dt);
        Eigen::Affine2d child_transform = translation * rotation;
        transform.transform = parent_transform->transform.matrix().inverse() * child_transform.matrix();

        return transform;
    } else {
        this->logger.warning("Transform of frame \"{}\" not given at TimePoint {}", child_frame, time);
        return std::nullopt;
    }
}
// NOLINTNEXTLINE(misc-no-recursion) - recursion gets cut when frames are the same
std::optional<Velocity> WorldModel::getVelocity(const std::string& child, const std::string& parent,
                                                const std::string& reference, time::TimePoint time,
                                                time::Duration averaging_interval, bool lock) const {
    std::shared_lock read_lock(this->transform_storage_mutex, std::defer_lock);
    auto time_copy = time;

    if (lock) {
        read_lock.lock();
    }

    // check for same frame
    if (child == parent) {
        Velocity velocity;
        velocity.header.child_frame = child;
        velocity.header.parent_frame = parent;
        velocity.header.stamp = time;
        velocity.velocity = Eigen::Vector3d::Zero();
        return velocity;
    }
    // check if child frame is empty so global frame will be used
    const std::string& child_frame = child.empty() ? this->global_frame : child;

    // check if parent frame is empty so global frame will be used
    const std::string& parent_frame = parent.empty() ? this->global_frame : parent;

    // check if reference frame is empty so global frame will be used
    const std::string& reference_frame = reference.empty() ? this->global_frame : reference;

    if (!this->transform_storage.contains(parent_frame)) {
        this->logger.warning("Parent frame \"\" not known! Cant calculate Velocity!", parent_frame);
        return std::nullopt;
    }

    auto child_iter = this->transform_storage.find(child_frame);

    if (child_iter == this->transform_storage.end()) {
        this->logger.warning("Child frame \"{}\" not known! Cant calculate Velocity!", child_frame);
        return std::nullopt;
    }
    if (!this->transform_storage.contains(reference_frame)) {
        this->logger.warning("Reference frame \"{}\" not known! Cant calculate Velocity!", reference_frame);
        return std::nullopt;
    }

    // get parent velocity
    std::optional<Velocity> parent_velocity =
        getVelocity(parent_frame, this->global_frame, reference_frame, time, averaging_interval, false);

    if (!parent_velocity) return std::nullopt;

    // check if velocity is provided
    auto& child_frame_transform = child_iter->second;
    auto transforms = getTransformElementBeforeAndAfterTimePoint(child_frame_transform, time);
    if (!transforms) return std::nullopt;

    auto [transform_before, transform_after] = *transforms;

    Velocity velocity;
    velocity.header.child_frame = child_frame;
    velocity.header.parent_frame = parent_frame;
    velocity.header.stamp = time;
    velocity.reference_frame = reference_frame;

    Eigen::Vector3d child_velocity;
    if (child_frame_transform.static_frame) {
        child_velocity = {0.0, 0.0, 0.0};
    } else if (transform_before.velocity && transform_after.velocity) {
        // velocities provided return interpolation of both velocities

        // interpolation
        // NOLINTNEXTLINE (bugprone-narrowing-conversions,cppcoreguidelines-narrowing-conversions)
        double dt = 0.0;
        if (transform_after.stamp != transform_before.stamp) {
            dt = (time - transform_before.stamp).count() /
                 static_cast<double>((transform_after.stamp - transform_before.stamp).count());
        }
        child_velocity = *transform_before.velocity + (*transform_after.velocity - *transform_before.velocity) * dt;
    } else {
        // no velocities given, need to calc with dx/dt
        std::optional<Transform> transform_end = getTransform(child_frame, this->global_frame, time_copy);
        if (!transform_end) return std::nullopt;
        std::optional<Transform> transform_start =
            getTransform(child_frame, this->global_frame, transform_end->header.stamp - averaging_interval);
        if (!transform_start) return std::nullopt;
        /// @todo handle 0
        double dt = time::Duration(transform_end->header.stamp - transform_start->header.stamp).asSec();
        Eigen::Vector3d x0 = {transform_start->transform.translation().x(),
                              transform_start->transform.translation().y(),
                              Eigen::Rotation2Dd(transform_start->transform.rotation()).angle()};
        Eigen::Vector3d x1 = {transform_end->transform.translation().x(), transform_end->transform.translation().y(),
                              Eigen::Rotation2Dd(transform_end->transform.rotation()).angle()};

        child_velocity.head(2) = (x1.head(2) - x0.head(2)) / dt;
        double rot_diff = x1.z() - x0.z();
        if (rot_diff > L_PI) {
            rot_diff -= 2 * L_PI;
        } else if (rot_diff < -L_PI) {
            rot_diff += 2 * L_PI;
        }
        child_velocity.z() = rot_diff / dt;
    }

    // transform into reference frame
    std::optional<Transform> reference_transform = getTransform(reference_frame, this->global_frame, time_copy);
    if (!reference_transform) return std::nullopt;

    child_velocity.head(2) = reference_transform->transform.rotation().inverse() * child_velocity.head(2);

    velocity.velocity = child_velocity - parent_velocity->velocity;

    return velocity;
}

[[nodiscard]] std::optional<AllyRobotData> WorldModel::getAllyRobotData(const RobotIdentifier& identifier,
                                                                        const time::TimePoint& time) const {
    if (this->robot_data.ally_robots.find(identifier) == this->robot_data.ally_robots.end()) return std::nullopt;

    if (time.asSec() == 0 && this->robot_data.ally_robots.at(identifier)->size() != 0) {
        return this->robot_data.ally_robots.at(identifier)->at(0);
    }

    for (size_t i = 0; i < this->robot_data.ally_robots.at(identifier)->size(); i++) {
        if (time >= this->robot_data.ally_robots.at(identifier)->at(i).time) {
            return this->robot_data.ally_robots.at(identifier)->at(i);
        }
    }

    size_t size = this->robot_data.ally_robots.at(identifier).get()->size();
    for (size_t i = 0; i < size; i++) {
        auto& data = this->robot_data.ally_robots.at(identifier)->at(i);
        this->logger.info("Buffer entry at timepoint: {} Ball in dribbler: {}", data.time, data.ball_in_dribbler);
    }

    if (size != 0) {
        this->logger.warning(
            "Requested time point ({}) for data of robot '{}' is to old, buffer not big enough! This can lead to huge "
            "performance problems. Check as soon as possible!",
            time, identifier);
    }

    return std::nullopt;
}

[[nodiscard]] std::optional<RobotData> WorldModel::getRobotData(const RobotIdentifier& identifier,
                                                                const time::TimePoint& time) const {
    if (identifier.isAlly() && this->robot_data.ally_robots.find(identifier) != this->robot_data.ally_robots.end()) {
        return this->getAllyRobotData(identifier, time);
    } else if (this->robot_data.enemy_robots.find(identifier) != this->robot_data.enemy_robots.end()) {
        if (time.asSec() == 0 && this->robot_data.enemy_robots.at(identifier)->size() != 0) {
            return this->robot_data.enemy_robots.at(identifier)->at(0);
        }

        auto size = this->robot_data.enemy_robots.at(identifier)->size();
        for (size_t i = 0; i < size; i++) {
            if (time > this->robot_data.enemy_robots.at(identifier)->at(i).time) {
                return this->robot_data.enemy_robots.at(identifier)->at(i);
            }
        }

        if (size != 0) {
            this->logger.warning(
                "Requested time point ({}) for data of robot '{}' is to old, buffer not big enough! This can lead to "
                "huge "
                "performance problems. Check as soon as possible!",
                time, identifier);
        }

        return std::nullopt;
    } else {
        return std::nullopt;
    }
}

bool WorldModel::pushEnemyRobotData(const RobotIdentifier& identifier, const RobotData& data) {
    if (!identifier.isEnemy() || this->robot_data.enemy_robots.find(identifier) == this->robot_data.enemy_robots.end())
        return false;
    if (this->robot_data.enemy_robots.at(identifier)->size() != 0 &&
        this->robot_data.enemy_robots.at(identifier)->at(0).time > data.time) {
        this->logger.warning("Data in Buffer for robot '{}' is newer! Cant push older AllyRobotData!", identifier);
        return false;
    }
    this->robot_data.enemy_robots.at(identifier)->push(data);
    return true;
}

bool WorldModel::pushAllyRobotData(const RobotIdentifier& identifier, const AllyRobotData& data) {
    if (!identifier.isAlly() || this->robot_data.ally_robots.find(identifier) == this->robot_data.ally_robots.end())
        return false;
    if (this->robot_data.ally_robots.at(identifier)->size() != 0 &&
        this->robot_data.ally_robots.at(identifier)->at(0).time > data.time) {
        this->logger.warning("Data in Buffer for robot '{}' is newer! Cant push older AllyRobotData!", identifier);
        return false;
    }
    this->robot_data.ally_robots.at(identifier)->push(data);
    return true;
}

bool WorldModel::removeRobotFromField(const RobotIdentifier& identifier) {
    TransformWithVelocity trans_with_vel;
    trans_with_vel.header.stamp = time::now();
    trans_with_vel.header.child_frame = identifier.getFrame();
    trans_with_vel.transform = OUT_OF_GAME_TRANSFORM;
    trans_with_vel.velocity = {0.0, 0.0, 0.0};
    if (!this->pushTransform(trans_with_vel)) return false;
    if (identifier.isAlly()) {
        AllyRobotData d;
        d.on_field = false;
        d.time = time::now();
        if (!this->pushAllyRobotData(identifier, d)) return false;
    } else {
        RobotData d;
        d.on_field = false;
        d.time = time::now();
        if (!this->pushEnemyRobotData(identifier, d)) return false;
    }
    return true;
}

std::vector<RobotIdentifier> WorldModel::getPossibleRobots() const { return this->robot_data.possible_robots; }

template <>
std::vector<RobotIdentifier> WorldModel::getPossibleRobots<Team::ALLY>() const {
    return this->robot_data.possible_ally_robots;
};

template <>
std::vector<RobotIdentifier> WorldModel::getPossibleRobots<Team::ENEMY>() const {
    return this->robot_data.possible_enemy_robots;
}

std::vector<RobotIdentifier> WorldModel::getVisibleRobots(const time::TimePoint& time) const {
    std::vector<RobotIdentifier> v;
    for (const auto& robot : this->robot_data.possible_robots) {
        auto robot_data = this->getRobotData(robot, time);
        if (robot_data && robot_data->on_field) {
            v.push_back(robot);
        }
    }
    return v;
}

template <>
std::vector<RobotIdentifier> WorldModel::getVisibleRobots<Team::ALLY>(const time::TimePoint& time) const {
    std::vector<RobotIdentifier> v;
    for (const auto& robot : this->robot_data.possible_ally_robots) {
        auto robot_data = this->getRobotData(robot, time);
        if (robot_data && robot_data->on_field) {
            v.push_back(robot);
        }
    }
    return v;
}

template <>
std::vector<RobotIdentifier> WorldModel::getVisibleRobots<Team::ENEMY>(const time::TimePoint& time) const {
    std::vector<RobotIdentifier> v;
    for (const auto& robot : this->robot_data.possible_enemy_robots) {
        auto robot_data = this->getRobotData(robot, time);
        if (robot_data && robot_data->on_field) {
            v.push_back(robot);
        }
    }
    return v;
}

[[nodiscard]] std::optional<GameState> WorldModel::getGameState(const time::TimePoint& time) const {
    if (time.asSec() == 0 && this->game_states->size() > 0) {
        return this->game_states->at(0).second;
    }

    for (size_t i = 0; i < this->game_states->size(); i++) {
        if (time >= this->game_states->at(i).first) {
            return this->game_states->at(i).second;
        }
    }
    this->logger.warning("Requested time point ({}) for a GameState is to old, buffer not big enough!", time);
    return std::nullopt;
}

bool WorldModel::pushNewGameState(const GameState& game_state, const time::TimePoint& time) {
    if (this->game_states->size() != 0 && this->game_states->at(0).first > time) {
        this->logger.warning("GameState in Buffer is newer! Cant push older GameState!");
        return false;
    }
    this->game_states->push({time, game_state});
    return true;
}

bool WorldModel::pushNewBallInfo(const BallInfo& ball_info) {
    if (this->ball_infos->size() != 0 && this->ball_infos->at(0).time > ball_info.time) {
        this->logger.warning("BallInfo in Buffer is newer! Cant push older GameState!");
        return false;
    }
    this->ball_infos->push(ball_info);
    return true;
}

bool WorldModel::updateBallPosition(const BallInfo& ball_info) {
    switch (ball_info.state) {
        case BallState::ON_FIELD: {
            if (!ball_info.position.has_value()) return false;
            TransformWithVelocity t;
            t.transform = ball_info.position->first;
            // t.velocity = ball_info.position->second;
            t.header.child_frame = this->ball_frame;
            t.header.parent_frame = this->global_frame;
            t.header.stamp = ball_info.time;
            return this->pushTransform(t);
        }
        case BallState::IN_ROBOT: {
            if (!ball_info.robot) return false;
            TransformWithVelocity t;
            t.transform = this->getTransform(ball_info.robot->getFrame(), "", ball_info.time)->transform;
            t.velocity = this->getVelocity(ball_info.robot->getFrame(), "", "", ball_info.time)->velocity;
            if (!t.transform || !t.velocity) return false;
            t.transform = t.transform.value().matrix() * ball_displacement_transform.matrix();
            t.header.child_frame = this->ball_frame;
            t.header.parent_frame = this->global_frame;
            t.header.stamp = ball_info.time;
            return this->pushTransform(t);
        }
        case BallState::MISSING: {
            TransformWithVelocity t;
            t.transform = OUT_OF_GAME_TRANSFORM;
            t.velocity = Eigen::Vector3d::Zero();
            t.header.child_frame = this->ball_frame;
            t.header.parent_frame = this->global_frame;
            t.header.stamp = ball_info.time;
            return this->pushTransform(t);
        }
    }
    return false;
}

[[nodiscard]] std::optional<BallInfo> WorldModel::getBallInfo(const time::TimePoint& time) const {
    if (time.asSec() == 0 && this->ball_infos->size() != 0) {
        return this->ball_infos->at(0);
    }

    for (size_t i = 0; i < this->ball_infos->size(); i++) {
        if (time >= this->ball_infos->at(i).time) {
            return this->ball_infos->at(i);
        }
    }
    this->logger.warning("Requested time point ({}) for a BallInfo is to old, buffer not big enough!", time);
    return std::nullopt;
}

[[nodiscard]] std::optional<Eigen::Vector2d> WorldModel::getBallPosition() const {
    auto elem = this->getTransform(this->ball_frame);
    if (!elem.has_value()) return std::nullopt;

    return elem->transform.translation();
}

[[nodiscard]] Eigen::Vector2d WorldModel::getBallPositionOr(const Eigen::Vector2d& default_value) const {
    auto pos = this->getBallPosition();
    if (!pos.has_value()) return default_value;
    return *pos;
}

[[nodiscard]] std::optional<Eigen::Vector3d> WorldModel::getBallVelocity() const {
    auto vel = this->getVelocity(this->ball_frame);
    if (!vel.has_value()) return std::nullopt;

    return vel->velocity;
}

[[nodiscard]] Eigen::Vector3d WorldModel::getBallVelocityOr(const Eigen::Vector3d& default_value) const {
    auto vel = this->getBallVelocity();
    if (!vel.has_value()) return default_value;
    return *vel;
}

}  // namespace luhsoccer::transform
