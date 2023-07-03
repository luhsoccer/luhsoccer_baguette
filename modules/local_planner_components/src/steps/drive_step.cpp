
#include <utility>

#include "local_planner_components/steps/drive_step.hpp"
#include "time/calc_time_stopwatch.hpp"
#include "logger/logger.hpp"

#include "local_planner_components/features/robot_cf_obstacle.hpp"
#include "local_planner_components/features/anti_target_feature.hpp"

#include "local_planner_components/shapes/rectangle_shape.hpp"
#include "local_planner_components/shapes/compose_shape.hpp"
#include "local_planner_components/features/anti_target_feature.hpp"
#include "local_planner_components/shapes/circle_shape.hpp"

#include "local_planner/avoidance_manager.hpp"

#include "drive_step_constraints/defense_area.hpp"
#include "drive_step_constraints/stop_state.hpp"
#include "drive_step_constraints/avoid_other_robots.hpp"
#include "drive_step_constraints/kickoff.hpp"
#include "drive_step_constraints/ball_placement.hpp"
#include "visit.hpp"
namespace luhsoccer::local_planner {

std::map<DriveStepConstraintNames, std::unique_ptr<DriveStepConstraint>> initConstraints() {
    std::map<DriveStepConstraintNames, std::unique_ptr<DriveStepConstraint>> map;
    map.emplace(DriveStepConstraintNames::STOP_STATE, std::make_unique<StopStateConstraint>());
    map.emplace(DriveStepConstraintNames::AVOID_OTHER_ROBOTS, std::make_unique<AvoidOtherRobotsConstraint>());
    map.emplace(DriveStepConstraintNames::KICKOFF_ENEMY, std::make_unique<Kickoff>(true));
    map.emplace(DriveStepConstraintNames::KICKOFF_ALLY, std::make_unique<Kickoff>(false));
    map.emplace(DriveStepConstraintNames::BALL_PLACEMENT, std::make_unique<BallPlacement>());
    map.emplace(DriveStepConstraintNames::DEFENSE_AREA, std::make_unique<DefenseAreaConstraint>());
    return map;
};

DriveStep::DriveStep(const std::vector<std::shared_ptr<const AbstractFeature>>& features,
                     std::shared_ptr<const AbstractRotationControl> rotation_control,
                     const ReachCondition& reach_condition, const BoolComponentParam& avoid_other_robots,
                     std::optional<DoubleComponentParam> feature_target_k_v,
                     std::optional<DoubleComponentParam> max_vel_x, std::optional<DoubleComponentParam> max_vel_y,
                     std::optional<DoubleComponentParam> max_vel_theta,
                     std::optional<DoubleComponentParam> step_drive_predict_duration_ms)
    : rotation_control(std::move(rotation_control)),
      reach_condition(reach_condition),
      feature_target_k_v(localPlannerConfig().feature_target_k_v),
      max_vel_x(localPlannerConfig().robot_vel_max_x),
      max_vel_y(localPlannerConfig().robot_vel_max_y),
      max_vel_theta(localPlannerConfig().robot_vel_max_theta),
      step_drive_predict_duration_ms(localPlannerConfig().step_drive_predict_duration_ms),
      robot_dynamic(1.0, 0.0),  // set friction to 0, because it is compensated by the robot
      constraints(initConstraints()) {
    this->setAvoidOtherRobots(avoid_other_robots);
    for (const auto& feature : features) {
        this->addFeatureImpl(feature);
    }
    if (feature_target_k_v) {
        this->feature_target_k_v = *feature_target_k_v;
    }
    if (max_vel_x) {
        this->max_vel_x = *max_vel_x;
    }
    if (max_vel_y) {
        this->max_vel_y = *max_vel_y;
    }
    if (max_vel_theta) {
        this->max_vel_theta = *max_vel_theta;
    }
    if (step_drive_predict_duration_ms) {
        this->step_drive_predict_duration_ms = *step_drive_predict_duration_ms;
    }
}

DriveStep::DriveStep()
    : reach_condition(ReachCondition::ONE_OF_TARGETS),
      feature_target_k_v(localPlannerConfig().feature_target_k_v),
      max_vel_x(localPlannerConfig().robot_vel_max_x),
      max_vel_y(localPlannerConfig().robot_vel_max_y),
      max_vel_theta(localPlannerConfig().robot_vel_max_theta),
      step_drive_predict_duration_ms(localPlannerConfig().step_drive_predict_duration_ms),
      robot_dynamic(1.0, 0.0),
      constraints(initConstraints()) {}

void DriveStep::setAvoidOtherRobots(const BoolComponentParam& avoid_other_robots) {
    this->constraints[DriveStepConstraintNames::AVOID_OTHER_ROBOTS]->setActive(avoid_other_robots);
};
void DriveStep::setAvoidDefenseArea(const BoolComponentParam& avoid_defense_area) {
    this->constraints[DriveStepConstraintNames::DEFENSE_AREA]->setActive(avoid_defense_area);
}
void DriveStep::setObeyStopState(const BoolComponentParam& obey_stop_state) {
    this->constraints[DriveStepConstraintNames::STOP_STATE]->setActive(obey_stop_state);
}
void DriveStep::activateConstraint(const DriveStepConstraintNames& name, const BoolComponentParam& active) {
    this->constraints[name]->setActive(active);
}

void DriveStep::addFeatureImpl(const std::shared_ptr<const AbstractFeature>& feature) {
    if (!feature) throw std::runtime_error("Added feature is null pointer!");
    switch (feature->getFeatureBaseType()) {
        case FeatureBaseType::TARGET: {
            // try to cast base feature to derived target feature
            std::shared_ptr<const AbstractTargetFeature> f =
                std::dynamic_pointer_cast<const AbstractTargetFeature>(feature);
            if (f) this->target_features.push_back(std::move(f));
            break;
        }
        case FeatureBaseType::CF_OBSTACLE: {
            // try to cast base feature to derived obstacle feature
            std::shared_ptr<const AbstractCFObstacle> f = std::dynamic_pointer_cast<const AbstractCFObstacle>(feature);

            if (f) this->cf_features.push_back(std::move(f));

            break;
        }
        default:
            throw std::runtime_error("Unhandled feature base type in drive step!");
    }
}

bool DriveStep::isReachedTranslational(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                                       const RobotIdentifier& robot, const time::TimePoint& time) const {
    if (this->reach_condition == ReachCondition::NEVER) return false;

    for (auto& target_feature : this->target_features) {
        if (target_feature->isReachable()) {
            if (this->reach_condition == ReachCondition::ONE_OF_TARGETS &&
                target_feature->isReached(wm, td, robot, time)) {
                return true;
            }
            if (this->reach_condition == ReachCondition::ALL_TARGETS &&
                !target_feature->isReached(wm, td, robot, time)) {
                return false;
            }
        }
    }
    // if function has not returned until now, all targets are reached and the reach condition is ALL_TARGETS ->
    // true or no target is reached and the reach condition is ONE_TARGET -> false
    return this->reach_condition == ReachCondition::ALL_TARGETS;
}

robot_interface::RobotCommand DriveStep::getStopCommand() const {
    robot_interface::RobotCommand null_cmd;
    robot_interface::RobotVelocityControl vel_cmd;
    vel_cmd.desired_velocity = {0.0, 0.0, 0.0};
    null_cmd.move_command = vel_cmd;
    return null_cmd;
}

std::optional<robot_interface::RobotCommand> DriveStep::calcRobotCommandFromForce(
    const Eigen::Vector3d& total_force, const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
    const RobotIdentifier& robot, const time::TimePoint time,
    const std::vector<std::pair<Eigen::Vector2d, std::optional<bool>>>& critical_obstacle_vectors) const {
    transform::RobotHandle robot_handle(robot, wm);
    robot_interface::RobotCommand command_msg;
    // current robot state
    std::optional<Eigen::Affine2d> robot_position = robot_handle.getPosition().getCurrentPosition(wm, "", time);
    if (!robot_position) return std::nullopt;
    std::optional<Eigen::Vector3d> robot_velocity = robot_handle.getPosition().getVelocity(wm, "", "", time);
    if (!robot_velocity) return std::nullopt;

    RobotState current_robot_state;
    current_robot_state.head(3) << robot_position->translation().x(), robot_position->translation().y(),
        Eigen::Rotation2Dd(robot_position->rotation()).angle();
    current_robot_state.tail(3) = *robot_velocity;

    RobotState desired_robot_state = this->robot_dynamic.applyAcceleration(
        current_robot_state, total_force, this->step_drive_predict_duration_ms.val(wm, td));

    // robot_interface::RobotPositionControl move_command;
    // move_command.desired_position = Eigen::Translation2d(desired_robot_state.x(), desired_robot_state.y()) *
    //                                 Eigen::Rotation2Dd(desired_robot_state.z());
    // move_command.desired_velocity = desired_robot_state.tail(3);
    robot_interface::RobotVelocityControl velocity_command;
    // desired_robot_state[5] = 0.0;
    Eigen::Vector3d desired_vel_global = desired_robot_state.tail(3);

    for (const auto& critical_vector : critical_obstacle_vectors) {
        if (critical_vector.first.dot(desired_vel_global.head(2)) < 0.0) {
            continue;
        }
        desired_vel_global *= std::min(1.0, localPlannerConfig().max_vel_at_collision / desired_vel_global.norm());
        Eigen::Vector2d normal = {critical_vector.first.y(), -critical_vector.first.x()};

        // desired_global_local.head(2) = normal.dot(desired_global_local.head(2)) / normal.squaredNorm() * normal;
        if (critical_vector.second.has_value()) {
            if (critical_vector.second.value()) {
                desired_vel_global.head(2) = normal.normalized() * desired_vel_global.norm();
            } else {
                desired_vel_global.head(2) = -normal.normalized() * desired_vel_global.norm();
            }
        } else {
            if (normal.dot(desired_vel_global.head(2)) > 0.0) {
                desired_vel_global.head(2) = normal.normalized() * desired_vel_global.norm();
            } else {
                desired_vel_global.head(2) = -normal.normalized() * desired_vel_global.norm();
            }
        }
    }
    Eigen::Vector3d desired_vel_local = desired_vel_global;
    desired_vel_local.head(2) =
        Eigen::Rotation2Dd(robot_position->rotation()).toRotationMatrix().inverse() * desired_vel_global.head(2);

    auto ball_info = wm->getBallInfo(time);
    if (ball_info.has_value() && ball_info->isInRobot(robot)) {
        // LOG_INFO(logger::Logger("DriveStep"), "Adding {:0.3f} m/s to y direction",
        //          localPlannerConfig().ball_y_vel_from_rotation * desired_vel_global.z());
        desired_vel_local.y() -= localPlannerConfig().ball_y_vel_from_rotation * desired_vel_global.z();
        desired_vel_local.x() += localPlannerConfig().ball_x_vel_from_rotation * std::abs(desired_vel_global.z());
        desired_vel_local.z() =
            RobotDynamic::clampVelocity(desired_vel_local.z(), localPlannerConfig().robot_vel_max_theta_with_ball);
    }

    if (this->constraints.at(DriveStepConstraintNames::STOP_STATE)->conditionMeet(wm, td)) {
        desired_vel_local.head(2) *=
            std::min(1.0, localPlannerConfig().stop_state_max_speed / desired_vel_local.head(2).norm());

        command_msg.dribbler_mode = robot_interface::DribblerMode::OFF;
    }
    if (desired_vel_local.head(2).norm() > localPlannerConfig().robot_moving_threshold) {
        desired_vel_local.z() =
            RobotDynamic::clampVelocity(desired_vel_local.z(), localPlannerConfig().robot_vel_max_theta_while_moving);
    }

    desired_vel_local.x() = RobotDynamic::clampVelocity(desired_vel_local.x(), this->max_vel_x.val(wm, td));
    desired_vel_local.y() = RobotDynamic::clampVelocity(desired_vel_local.y(), this->max_vel_y.val(wm, td));
    desired_vel_local.z() = RobotDynamic::clampVelocity(desired_vel_local.z(), this->max_vel_theta.val(wm, td));

    velocity_command.desired_velocity = desired_vel_local;
    // LOG_INFO(logger::Logger("DriveSetp"), "velocity_command.desired_velocity: {}",
    // velocity_command.desired_velocity);
    command_msg.move_command = velocity_command;

    // visualize rotation values
    if (config_provider::ConfigProvider::getConfigStore().local_planner_visualization_config.display_rotation_vlc) {
        auto marker = this->getCookie<marker::LinePlot>(td, "rotation_plot");
        if (!marker.has_value()) {
            marker = marker::LinePlot("rotation_plot");
        }

        marker::LinePlot::LineHandle currentVelocityLine = marker->getLine("current velocity");
        marker::LinePlot::LineHandle forceLine = marker->getLine("force");
        marker::LinePlot::LineHandle commandedVelocity = marker->getLine("commaned velocity");
        // NOLINTBEGIN
        marker->addPoint(currentVelocityLine, time::now().asSec(), robot_velocity->z());
        marker->addPoint(forceLine, time::now().asSec(), total_force.z() * 10);
        marker->addPoint(commandedVelocity, time::now().asSec(), desired_vel_local.z());
        // NOLINTEND
        this->setCookie(td, "rotation_plot", marker.value());
    }
    if (config_provider::ConfigProvider::getConfigStore().local_planner_visualization_config.plot_translation_vlc) {
        auto marker = this->getCookie<marker::LinePlot>(td, "translation_plot");
        if (!marker.has_value()) {
            marker = marker::LinePlot("translation_plot");
        }
        marker::LinePlot::LineHandle currentVelocityLine = marker->getLine("current velocity");
        marker::LinePlot::LineHandle forceLine = marker->getLine("force");
        marker::LinePlot::LineHandle commandedVelocity = marker->getLine("commaned velocity");
        // NOLINTBEGIN
        marker->addPoint(currentVelocityLine, time::now().asSec(), robot_velocity->head(2).norm());
        marker->addPoint(forceLine, time::now().asSec(), total_force.head(2).norm() * 10);
        marker->addPoint(commandedVelocity, time::now().asSec(), desired_vel_local.head(2).norm());
        // NOLINTEND
        this->setCookie(td, "translation_plot", marker.value());
    }
    return command_msg;
}

double DriveStep::calcTargetRelaxationFactors(const std::shared_ptr<const transform::WorldModel>& /*wm*/,
                                              const TaskData& /*td*/,
                                              const std::optional<Eigen::Vector2d>& min_obstacle_vec,
                                              const Eigen::Vector2d& max_target_vec) const {
    if (!min_obstacle_vec.has_value()) return 1.0;
    double w1 = 1.0 - std::exp(-min_obstacle_vec->norm() /
                               (localPlannerConfig().robot_radius * localPlannerConfig().step_drive_w1_alpha));
    double w2 =
        1.0 - (max_target_vec.dot(min_obstacle_vec.value()) / (max_target_vec.norm() * min_obstacle_vec->norm()));

    return std::max(w1 * w2, localPlannerConfig().setp_drive_min_relaxation_factor.val());
};

std::pair<AbstractStep::StepState, robot_interface::RobotCommand> DriveStep::calcCommandMessage(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
    const std::shared_ptr<AvoidanceManager>& am, const time::TimePoint time) const {
    transform::RobotHandle robot_handle(robot, wm);

    Eigen::Vector2d total_artificial_desired_velocity = Eigen::Vector2d::Zero();

    Eigen::Vector2d max_target_vec = Eigen::Vector2d::Zero();
    Eigen::Vector2d sum_weighted_target = Eigen::Vector2d::Zero();
    double sum_target_weights = 0.0;

    std::map<size_t, bool> influence_map;
    // add desired velocity from all targets
    // ws.startSection("target-force");

    std::vector<std::shared_ptr<const AbstractTargetFeature>> current_target_features = this->target_features;

    for (const auto& constraint : this->constraints) {
        if (constraint.second->conditionMeet(wm, td)) {
            auto additional_target_features = constraint.second->getAdditionalTargetFeatures();
            current_target_features.insert(current_target_features.end(), additional_target_features.begin(),
                                           additional_target_features.end());
        }
    }

    double weight_sum = 0.0;
    for (auto& target_feature : current_target_features) {
        auto desired_vel = target_feature->calcArtificialDesiredVelocity(wm, td, robot, time);
        total_artificial_desired_velocity += desired_vel;
        bool reached = target_feature->isReached(wm, td, robot, time);
        bool influence = !reached && !desired_vel.isZero();
        influence_map[target_feature->getUid()] = influence;
        if (influence) weight_sum += target_feature->getWeight(wm, td);
        // calc target distances
        auto transform_to_shape =
            target_feature->getShape()->getTransformToClosestPoint(wm, td, robot_handle.getPosition(), time, "");
        if (transform_to_shape.vec && influence) {
            if (transform_to_shape.vec->norm() > max_target_vec.norm()) max_target_vec = *transform_to_shape.vec;

            sum_weighted_target += transform_to_shape.vec.value() * target_feature->getWeight(wm, td);
            sum_target_weights += target_feature->getWeight(wm, td);
        }
    }
    // determine mean weighted target
    Eigen::Vector2d mean_weighted_target = Eigen::Vector2d::Zero();
    if (abs(sum_target_weights) > 0.0) {
        mean_weighted_target = sum_weighted_target / sum_target_weights;
    }
    if (weight_sum != 0.0) {
        total_artificial_desired_velocity /= weight_sum;
        // limit to maximum forward velocity of robot
    }
    total_artificial_desired_velocity *=
        std::min(1.0, this->max_vel_x.val(wm, td) / total_artificial_desired_velocity.norm());

    // get current velocity of robot
    std::optional<Eigen::Vector3d> current_velocity = robot_handle.getPosition().getVelocity(wm, "", "", time);
    if (!current_velocity) return {AbstractStep::StepState::ERROR, robot_interface::RobotCommand()};

    // calc force from targets
    Eigen::Vector2d target_force =
        this->feature_target_k_v.val(wm, td) * (total_artificial_desired_velocity - current_velocity->head(2));

    // get rotation vectors
    std::vector<std::shared_ptr<const AbstractCFObstacle>> current_cf_features(this->cf_features);

    for (const auto& constraint : this->constraints) {
        if (constraint.second->conditionMeet(wm, td)) {
            auto additional_obstacle_features = constraint.second->getAdditionalObstacleFeatures();
            current_cf_features.insert(current_cf_features.end(), additional_obstacle_features.begin(),
                                       additional_obstacle_features.end());
        }
    }

    // remove features with weight of 0
    current_cf_features.erase(std::remove_if(current_cf_features.begin(), current_cf_features.end(),
                                             [&wm, &td](const std::shared_ptr<const AbstractCFObstacle>& feature) {
                                                 constexpr double SMALL_VALUE = 0.0001;
                                                 return feature->getWeight(wm, td) < SMALL_VALUE;
                                             }),
                              current_cf_features.end());

    AvoidForceResult result =
        am->getTotalForce(wm, td, robot, time, current_cf_features, target_force, max_target_vec, mean_weighted_target);
    influence_map.merge(result.influence_map);

    Eigen::Vector2d total_translational_force = result.total_force;
    // determine rotation force
    double rotation_force = 0.0;
    bool rotation_reached = true;
    if (!this->ignore_rotation.val(wm, td)) {
        assert(this->rotation_control);  // No Rotation Control defined in DriveStep!
        auto res = this->rotation_control->calcRotationForce(wm, td, robot);
        rotation_force = res.first;
        rotation_reached = res.second;
    }

    Eigen::Vector3d total_force = {total_translational_force.x(), total_translational_force.y(), rotation_force};

    // determine velocity and position command from force
    auto command_msg = this->calcRobotCommandFromForce(total_force, wm, td, robot, time, result.critical_vectors);
    if (!command_msg.has_value()) return {AbstractStep::StepState::ERROR, robot_interface::RobotCommand()};
    // ws.endSection("calc-robot-command");

    if ((rotation_reached && this->isReachedTranslational(wm, td, robot, time)) || this->cancel_condition.val(wm, td)) {
        return {AbstractStep::StepState::FINISHED, this->getStopCommand()};
    }
    // ws.endSection("step");
    // ws.printSectionTimes();
    this->setCookie(td, "influence_map", influence_map);
    this->setCookie(td, "magnetic_field_vec_map", result.magnetic_field_vec_map);
    return {AbstractStep::StepState::RUNNING, command_msg.value()};
}

std::vector<Marker> DriveStep::getVisualizationMarkers(const std::shared_ptr<const transform::WorldModel>& wm,
                                                       const TaskData& td, const RobotIdentifier& robot,
                                                       const time::TimePoint time) const {
    std::vector<std::vector<Marker>> markers;
    auto influence_map = this->getCookie<std::map<size_t, bool>>(td, "influence_map");
    auto magnetic_field_vec_map = this->getCookie<std::map<size_t, bool>>(td, "magnetic_field_vec_map");
    const auto& viz_options = config_provider::ConfigProvider::getConfigStore().local_planner_visualization_config;

    static const auto VIS_INFLUENCE = [&viz_options](
                                          const std::optional<std::map<size_t, bool>>& influence_map,
                                          const std::optional<std::map<size_t, bool>>& magnetic_field_vec_map,
                                          std::vector<Marker>& ms, size_t uid) {
        if (!viz_options.shapes_display_influence) return;
        if (influence_map.has_value()) {
            auto influence_it = influence_map->find(uid);
            if (influence_it != influence_map->end()) {
                if (!influence_it->second) {
                    // has no influence
                    for (auto& m : ms) {
                        auto visitor = overload{[&viz_options](marker::Marker& marker) {
                            marker::Color c = marker.getColor();
                            c.alpha = viz_options.shapes_influence_value;
                            marker.setColor(c);
                        }};
                        std::visit(visitor, m);
                    }
                }
            }
        }
        if (magnetic_field_vec_map.has_value()) {
            auto magnetic_field_vec_it = magnetic_field_vec_map->find(uid);
            if (magnetic_field_vec_it != magnetic_field_vec_map->end()) {
                if (magnetic_field_vec_it->second) {
                    for (auto& m : ms) {
                        auto visitor = overload{[](marker::Marker& marker) { marker.setColor(marker::Color::BLUE()); }};
                        std::visit(visitor, m);
                    }
                } else {
                    for (auto& m : ms) {
                        auto visitor =
                            overload{[](marker::Marker& marker) { marker.setColor(marker::Color::YELLOW()); }};
                        std::visit(visitor, m);
                    }
                }
            }
        }
    };
    for (auto& feature : this->target_features) {
        auto m = feature->getVisualizationMarker(wm, td, robot, time);
        VIS_INFLUENCE(influence_map, magnetic_field_vec_map, m, feature->getUid());
        markers.push_back(m);
    }

    for (auto& feature : this->cf_features) {
        auto m = feature->getVisualizationMarker(wm, td, robot, time);
        VIS_INFLUENCE(influence_map, magnetic_field_vec_map, m, feature->getUid());
        markers.push_back(m);
    }

    for (const auto& constraint : this->constraints) {
        if (constraint.second->conditionMeet(wm, td)) {
            auto additional_target_features = constraint.second->getAdditionalTargetFeatures();
            for (const auto& feature : additional_target_features) {
                auto m = feature->getVisualizationMarker(wm, td, robot, time);
                VIS_INFLUENCE(influence_map, magnetic_field_vec_map, m, feature->getUid());
                markers.push_back(m);
            }

            auto additional_obstacle_features = constraint.second->getAdditionalObstacleFeatures();
            for (const auto& feature : additional_obstacle_features) {
                auto m = feature->getVisualizationMarker(wm, td, robot, time);
                VIS_INFLUENCE(influence_map, magnetic_field_vec_map, m, feature->getUid());
                markers.push_back(m);
            }
        }
    }

    auto rotation_vector_map = this->getCookie<std::map<size_t, bool>>(td, "rotation_vector_map");
    // avoid other robots
    if (this->rotation_control) {
        markers.push_back(this->rotation_control->getVisualizationMarker(wm, td, robot, time));
    }

    std::vector<Marker> all_markers;
    for (const auto& ms : markers) all_markers.insert(all_markers.end(), ms.begin(), ms.end());

    for (auto& m : all_markers) {
        constexpr double SLIGHTLY_ABOVE_GROUND = 0.02;
        auto visitor = overload{[](marker::Marker& marker) { marker.setHeight(SLIGHTLY_ABOVE_GROUND); }};
        std::visit(visitor, m);
    }

    auto rotation_marker = this->getCookie<marker::LinePlot>(td, "rotation_plot");
    auto translation_marker = this->getCookie<marker::LinePlot>(td, "translation_plot");

    if (rotation_marker.has_value()) all_markers.emplace_back(rotation_marker.value());
    if (translation_marker.has_value()) all_markers.emplace_back(translation_marker.value());

    return all_markers;
}

}  // namespace luhsoccer::local_planner