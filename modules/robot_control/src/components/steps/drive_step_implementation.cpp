#include "config/robot_control_visualization_config.hpp"
#include "cooperation_module.hpp"
#include "marker_adapter.hpp"
#include "robot_control/components/steps/drive_step.hpp"
#include "components/steps/drive_step_constraint.hpp"
#include "robot_control/components/component_data.hpp"
#include "config/robot_control_config.hpp"

#include <utility>

namespace luhsoccer::robot_control {

robot_interface::RobotCommand DriveStep::getStopCommand() {
    robot_interface::RobotCommand null_cmd;
    robot_interface::RobotVelocityControl vel_cmd;
    vel_cmd.desired_velocity = {0.0, 0.0, 0.0};
    null_cmd.move_command = vel_cmd;
    return null_cmd;
}

bool DriveStep::isReachedTranslational(const ComponentData& comp_data) const {
    if (this->reach_condition == ReachCondition::NEVER) return false;

    for (auto& target_feature : this->target_features) {
        if (target_feature->isReachable()) {
            if (this->reach_condition == ReachCondition::ONE_OF_TARGETS && target_feature->isReached(comp_data)) {
                return true;
            }
            if (this->reach_condition == ReachCondition::ALL_TARGETS && !target_feature->isReached(comp_data)) {
                return false;
            }
        }
    }
    // if function has not returned until now, all targets are reached and the reach condition is ALL_TARGETS ->
    // true or no target is reached and the reach condition is ONE_TARGET -> false
    return this->reach_condition == ReachCondition::ALL_TARGETS;
}

void DriveStep::displayPlots(const ComponentData& comp_data, const Eigen::Vector3d& robot_velocity,
                             const Eigen::Vector3d& cmd_velocity) const {
    if (robotControlVisualizationConfig().plot_translational) {
        auto plt = this->getCookie<marker::LinePlot>(comp_data.td, "translation_plot");
        if (!plt.has_value()) {
            plt = marker::LinePlot(fmt::format("Translation Plot {}", comp_data.robot));
        }

        marker::LinePlot::LineHandle velocity = plt->getLine("Current velocity");
        marker::LinePlot::LineHandle cmd = plt->getLine("Commanded Velocity");
        auto time = comp_data.time;
        if (time.asNSec() == 0) {
            time = time::now();
        }
        // NOLINTBEGIN (cppcoreguidelines-narrowing-conversions)
        plt->addPoint(velocity, time.asSec(), robot_velocity.head(2).norm());
        plt->addPoint(cmd, time.asSec(), cmd_velocity.head(2).norm());
        // NOLINTEND
        this->setCookie(comp_data.td, "translation_plot", plt.value());
        comp_data.ma.displayMarker(plt.value());
    }

    if (robotControlVisualizationConfig().plot_rotational) {
        auto plt = this->getCookie<marker::LinePlot>(comp_data.td, "rotation_plot");
        if (!plt.has_value()) {
            plt = marker::LinePlot(fmt::format("Rotation Plot {}", comp_data.robot));
        }

        marker::LinePlot::LineHandle velocity = plt->getLine("Current velocity");
        marker::LinePlot::LineHandle cmd = plt->getLine("Commanded Velocity");
        auto time = comp_data.time;
        if (time.asNSec() == 0) {
            time = time::now();
        }
        // NOLINTBEGIN (cppcoreguidelines-narrowing-conversions)
        plt->addPoint(velocity, time.asSec(), robot_velocity.z());
        plt->addPoint(cmd, time.asSec(), cmd_velocity.z());
        // NOLINTEND
        this->setCookie(comp_data.td, "rotation_plot", plt.value());
        comp_data.ma.displayMarker(plt.value());
    }
}

double calcTargetRelaxationFactors(const Eigen::Vector2d& min_obstacle_vec, const Eigen::Vector2d& max_target_vec) {
    double w1 = 1.0 - robotControlConfig().rel_factors_w1_k *
                          std::exp(-min_obstacle_vec.norm() /
                                   (robotControlConfig().robot_radius * robotControlConfig().rel_factors_w1_alpha));
    double w2 = 1.0 - robotControlConfig().rel_factors_w2_k *
                          (max_target_vec.dot(min_obstacle_vec) / (max_target_vec.norm() * min_obstacle_vec.norm()));

    return std::max(w1 * w2, robotControlConfig().rel_factors_min.val());
};

std::pair<AbstractStep::StepState, robot_interface::RobotCommand> DriveStep::calcCommandMessage(
    const ComponentData& comp_data) const {
    // ---------------------------------------------------- Active Features --------------------------------------------
    std::vector<std::shared_ptr<const AbstractObstacle>> current_obstacles = this->cf_features;
    std::vector<std::shared_ptr<const AbstractShape>> restricted_areas;

    for (const auto& constraint : this->constraints) {
        if (constraint.second->conditionMet(comp_data)) {
            auto additional_obstacle_features = constraint.second->getAdditionalObstacleFeatures();
            current_obstacles.insert(current_obstacles.end(), additional_obstacle_features.begin(),
                                     additional_obstacle_features.end());
        }
    }

    // remove features with weight of 0
    current_obstacles.erase(std::remove_if(current_obstacles.begin(), current_obstacles.end(),
                                           [&comp_data](const std::shared_ptr<const AbstractObstacle>& feature) {
                                               constexpr double SMALL_VALUE = 0.0001;
                                               return feature->getWeight(comp_data) < SMALL_VALUE;
                                           }),
                            current_obstacles.end());

    for (const auto& obstacle : current_obstacles) {
        if (obstacle->isRestrictedArea(comp_data) && obstacle->getShape()->hasArea(comp_data)) {
            restricted_areas.push_back(obstacle->getShape());
        }
    }

    std::vector<std::shared_ptr<const AbstractTargetFeature>> current_target_features = this->target_features;

    transform::Position robot_position(comp_data.robot.getFrame());

    // add constraint target features
    for (const auto& constraint : this->constraints) {
        if (constraint.second->conditionMet(comp_data)) {
            auto additional_target_features = constraint.second->getAdditionalTargetFeatures();
            current_target_features.insert(current_target_features.end(), additional_target_features.begin(),
                                           additional_target_features.end());
        }
    }

    // ---------------------------------------------------- Targets ----------------------------------------------------

    Eigen::Vector2d target_cmd_vel = Eigen::Vector2d::Zero();

    Eigen::Vector2d max_target_vec = Eigen::Vector2d::Zero();
    Eigen::Vector2d sum_weighted_target = Eigen::Vector2d::Zero();
    double sum_target_weights = 0.0;

    double weight_sum = 0.0;
    for (auto& target_feature : current_target_features) {
        auto [desired_vel, vec_to_target] =
            target_feature->calcArtificialDesiredVelocity(comp_data, restricted_areas, robot_position);
        target_cmd_vel += desired_vel;
        bool reached = target_feature->isReached(comp_data);
        bool influence = !reached && !desired_vel.isZero();
        // calc target distances
        // auto transform_to_shape = target_feature->getTransformToClosestPointFromShape(comp_data, robot_position);
        if (influence) {
            weight_sum += target_feature->getWeight(comp_data);

            if (target_feature->isReachable()) {
                if (vec_to_target.norm() > max_target_vec.norm()) max_target_vec = vec_to_target;
                sum_weighted_target += vec_to_target * target_feature->getWeight(comp_data);
                sum_target_weights += target_feature->getWeight(comp_data);
            }
        }
    }
    // determine mean weighted target
    Eigen::Vector2d mean_weighted_target = Eigen::Vector2d::Zero();
    if (abs(sum_target_weights) > 0.0) {
        mean_weighted_target = sum_weighted_target / sum_target_weights;
    }
    if (weight_sum != 0.0) {
        target_cmd_vel /= weight_sum;
        // limit to maximum forward velocity of robot
    }

    /// @todo use y and x max velocity
    target_cmd_vel *= std::min(1.0, this->max_vel_x.val(comp_data) / target_cmd_vel.norm());

    // ---------------------------------------------------- Obstacles --------------------------------------------------
    // get rotation vectors

    Eigen::Vector2d obstacle_cmd_vel = Eigen::Vector2d::Zero();
    std::optional<Eigen::Vector2d> min_obstacle_vector;

    auto rotation_vectors = comp_data.coop.getRotationVectors(comp_data, current_obstacles, mean_weighted_target);
    size_t i = 0;

    // We subtract 3cm from the mean target distance, so that if the robot drive to an obstacle brim, this obstacle will
    // not influence the robot when approaching.
    constexpr double MEAN_OFFSET_FOR_INFLUENCE = 0.03;

    std::vector<std::pair<Eigen::Vector2d, bool>> collision_avoid_vectors;
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> breaking_obstacle_vectors;

    for (const auto& obstacle : current_obstacles) {
        auto obstacle_velocity =
            obstacle->getVelocityCommand(comp_data, target_cmd_vel, rotation_vectors[i],
                                         mean_weighted_target.norm() - MEAN_OFFSET_FOR_INFLUENCE, robot_position, "");

        auto res = obstacle->getVecAndVelocity(comp_data, robot_position, "");
        if (res.has_value()) {
            if (obstacle_velocity.has_value()) {
                obstacle_cmd_vel += obstacle_velocity.value();

                if (!min_obstacle_vector.has_value() || res->first.norm() < min_obstacle_vector->norm()) {
                    min_obstacle_vector = res->first;
                }
            }
            if (res->first.norm() < std::min(obstacle->getInfluenceDistance(comp_data),
                                             mean_weighted_target.norm() - MEAN_OFFSET_FOR_INFLUENCE)) {
                if (obstacle->avoidCollision(comp_data) &&
                    res->first.norm() < obstacle->getCriticalDistance(comp_data)) {
                    collision_avoid_vectors.emplace_back(res->first, rotation_vectors[i]);
                }
            }
            if (obstacle->breakRobotTowardsObstacle(comp_data)) {
                auto obstacle_vel = obstacle->getTransformToClosestPointFromShape(comp_data, "");
                if (obstacle_vel.velocity.has_value()) {
                    breaking_obstacle_vectors.emplace_back(res->first, obstacle_vel.velocity.value());
                }
            }
        }
        i++;
    }

    double relaxation_factor = 1.0;
    if (min_obstacle_vector.has_value()) {
        relaxation_factor = calcTargetRelaxationFactors(min_obstacle_vector.value(), max_target_vec);
    }

    Eigen::Vector2d total_command_velocity = relaxation_factor * target_cmd_vel + obstacle_cmd_vel;

    // Limit total command velocity to original target vector length
    // Since the obstacle cmd vel is perpendicular to the cmd vel form targets the obstacle cmd vel the total cmd vector
    // can only get bigger and then is limited to the original length. By that the obstacle cmd vel cannot lower the
    // target cmd vel.
    total_command_velocity *= std::min(1.0, target_cmd_vel.norm() / total_command_velocity.norm());

    // -------------------------------------------- Collision prevention -----------------------------------------------

    // break robot towards obstacles
    for (const auto& [obstacle_vector, obstacle_vel] : breaking_obstacle_vectors) {
        double max_vel_in_obstacle_direction =
            robotControlConfig().target_k_p * robotControlConfig().obstacle_max_vel_k * obstacle_vector.norm();

        double velocity_in_direction = total_command_velocity.dot(obstacle_vector) / obstacle_vector.norm();
        double obstacle_vel_in_direction = obstacle_vel.dot(obstacle_vector) / obstacle_vector.norm();
        velocity_in_direction -= obstacle_vel_in_direction;

        if (velocity_in_direction > max_vel_in_obstacle_direction)
            total_command_velocity +=
                obstacle_vector.normalized() * (max_vel_in_obstacle_direction - velocity_in_direction);
    }

    // collision avoidance
    std::vector<Eigen::Vector2d> checked_vectors;
    checked_vectors.reserve(collision_avoid_vectors.size());
    for (const auto& [critical_vector, bypass_dir] : collision_avoid_vectors) {
        if (critical_vector.dot(total_command_velocity) < 0.0) {
            checked_vectors.push_back(critical_vector);
            continue;
        }
        total_command_velocity *=
            std::min(1.0, robotControlConfig().obstacle_collision_max_vel / total_command_velocity.norm());
        Eigen::Vector2d normal = {critical_vector.y(), -critical_vector.x()};
        if (!bypass_dir) normal *= -1;

        // check if preferred normal is in previously checked critical vector range
        bool collides_with_previous =
            std::any_of(checked_vectors.begin(), checked_vectors.end(),
                        [&normal](const Eigen::Vector2d& checked) { return checked.dot(normal) > 0.0; });
        if (collides_with_previous) normal *= -1;

        total_command_velocity = normal.normalized() * total_command_velocity.norm();

        auto pos = robot_position.getCurrentPosition(comp_data.wm, "", comp_data.time);
        if (pos.has_value()) {
            auto robot_trans = pos->translation();

            marker::Line l("");
            l.setLinePoints({robot_trans.x(), robot_trans.y()},
                            {critical_vector.x() + robot_trans.x(), critical_vector.y() + robot_trans.y()});
            l.setColor(RC_RED);
            comp_data.ma.displayMarker(l);
        }

        checked_vectors.push_back(critical_vector);
    }

    // ---------------------------------------------------- Rotation ---------------------------------------------------

    double rotation_cmd = 0.0;
    bool rotation_reached = true;
    if (!this->ignore_rotation.val(comp_data)) {
        assert(this->rotation_control);  // No Rotation Control defined in DriveStep!
        auto res = this->rotation_control->calcRotationVelocity(comp_data, robot_position);
        rotation_cmd = res.first;
        rotation_reached = res.second;
    }

    if ((rotation_reached && this->isReachedTranslational(comp_data)) || this->cancel_condition.val(comp_data)) {
        return {AbstractStep::StepState::FINISHED, this->getStopCommand()};
    }

    robot_interface::RobotCommand command_msg;

    auto robot_pose = robot_position.getCurrentPosition(comp_data.wm, "", comp_data.time);
    if (!robot_pose.has_value()) return {AbstractStep::StepState::ERROR, this->getStopCommand()};
    Eigen::Vector2d local_velocity_command = robot_pose->rotation().inverse() * total_command_velocity;
    // ---------------------------------------------------- limit velocity ---------------------------------------------

    // if robot has ball
    auto ball_info = comp_data.wm->getBallInfo(comp_data.time);
    if (ball_info.has_value() && ball_info->isInRobot(comp_data.robot)) {
        local_velocity_command.x() += robotControlConfig().turn_with_ball_x_vel_from_rotation * std::abs(rotation_cmd);
        local_velocity_command.y() -= robotControlConfig().turn_with_ball_y_vel_from_rotation * rotation_cmd;
        rotation_cmd *= std::min(1.0, robotControlConfig().turn_around_ball_max_vel_theta / abs(rotation_cmd));
    }

    // if in stop state
    if (this->constraints.at(DriveStepConstraintNames::STOP_STATE)->conditionMet(comp_data)) {
        local_velocity_command *=
            std::min(1.0, robotControlConfig().stop_state_max_vel / local_velocity_command.norm());

        command_msg.dribbler_mode = robot_interface::DribblerMode::OFF;
    }

    // ---------------------------------------------------- Robot Command ----------------------------------------------

    command_msg.time_sent = comp_data.time;

    robot_interface::RobotVelocityControl vel_control;
    vel_control.desired_velocity = {local_velocity_command.x(), local_velocity_command.y(), rotation_cmd};
    command_msg.move_command = vel_control;

    auto robot_velocity_vec = robot_position.getVelocity(comp_data.wm, "", "", comp_data.time);
    if (robot_velocity_vec.has_value())
        this->displayPlots(comp_data, robot_velocity_vec.value(), vel_control.desired_velocity);

    return {AbstractStep::StepState::RUNNING, command_msg};
}
}  // namespace luhsoccer::robot_control