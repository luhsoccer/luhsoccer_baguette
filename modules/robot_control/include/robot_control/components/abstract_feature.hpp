/**
 * @file abstract_feature.hpp
 * @author Fabrice Zeug (fabrice.zeug@luhbots.de)
 * @brief
 * @version 0.1
 * @date 2022-10-07
 *
 * @copyright Copyright (c) 2022
 *
 */
#pragma once

#include <utility>

#include "robot_control/components/abstract_shape.hpp"
#include "robot_control/components/component_util.hpp"
#include "robot_control/components/abstract_component.hpp"
#include "marker_service/marker.hpp"

namespace luhsoccer::robot_control {

/// @brief enum of all subtypes of features
enum class FeatureBaseType { TARGET, OBSTACLE };

/**
 * @brief The AbstractFeature class implements an abstract base class for all Feature types.
 * Features are all (virtual) objects that influence the translational driving of a robot during a skill. The feature
 * itself does not include any function to return any kind of driving command, as features a split in two subtypes:
 * TargetFeatures and ObstacleFeatures. These subtypes define an abstract calculation function themselves. The
 * AbstractFeature class only implements universal feature utility, i.e., markers and shapes
 */
class AbstractFeature : public AbstractComponent {
   public:
    ~AbstractFeature() override = default;
    AbstractFeature(const AbstractFeature&) = default;
    AbstractFeature& operator=(const AbstractFeature&) = default;
    AbstractFeature(AbstractFeature&&) = default;
    AbstractFeature& operator=(AbstractFeature&&) = default;

    /**
     * @brief Get the current Weight of this feature
     *
     * @param wm WorldModel
     * @param td current TaskData
     * @return double current Weight
     */
    [[nodiscard]] double getWeight(const ComponentData& comp_data) const;

    /**
     * @brief Get the Shape
     *
     * @return const std::shared_ptr<const AbstractShape>& shape of feature
     */
    [[nodiscard]] const std::shared_ptr<const AbstractShape>& getShape() const { return this->shape; }

    /**
     * @brief Get the Feature Base Type
     *
     * @return FeatureBaseType Feature base type of this feature
     */
    [[nodiscard]] virtual constexpr FeatureBaseType getFeatureBaseType() const = 0;

    /**
     * @brief Get the Visualization Color
     * Returns the color of the marker of this feature.
     * This function has to be implemented by every feature type.
     * @param wm WorldModel
     * @param td TaskData
     * @param robot robot
     * @param time time point
     * @return marker::Color color of the marker
     */
    [[nodiscard]] virtual marker::Color getVisualizationColor(const ComponentData& comp_data) const = 0;

    void visualizeAdditionalOptions(const ComponentData& comp_data) const;

    [[nodiscard]] VectorWithVelocityStamped getTransformToClosestPointFromShape(
        const ComponentData& comp_data, const transform::Position& robot_position,
        const transform::Position& observe_position = "") const;

   protected:
    /**
     * @brief Construct a new Abstract Feature object
     *
     * @param shape shape of feature
     * @param weight weight of the feature
     * @param max_vel_x optional custom max velocity in x direction
     * @param max_vel_y optional custom max velocity in y direction
     */
    AbstractFeature(std::shared_ptr<const AbstractShape> shape, DoubleComponentParam weight = 1.0);

    /// @brief the weight of this feature. The weight is a multiplier for the commands of this feature.
    DoubleComponentParam weight;

   private:
    /// @brief shape of this feature
    std::shared_ptr<const AbstractShape> shape;

    // configuration methods
   public:
    void setWeight(DoubleComponentParam weight) { this->weight = std::move(weight); }
};

/**
 * @brief The AbstractTargetFeature implements a base target feature type.
 * Target features implement the 'calcArtificialDesiredVelocity' which returns the a desired velocity of the robot that
 * is caused by this feature.
 */
class AbstractTargetFeature : public AbstractFeature {
   public:
    /// @brief return that this feature is a target feature
    [[nodiscard]] constexpr FeatureBaseType getFeatureBaseType() const override { return FeatureBaseType::TARGET; };
    /**
     * @brief calculate the current desired translational velocity of the robot caused by this feature.
     *
     * @param wm WorldModel
     * @param td TaskData
     * @param robot robot
     * @param time time point
     * @param observe_position coordinate system in which the desired velocity is given. Only the rotation is
     * significant
     * @return Eigen::Vector2d current desired translational velocity caused by this feature
     */
    [[nodiscard]] virtual std::pair<Eigen::Vector2d, Eigen::Vector2d> calcArtificialDesiredVelocity(
        const ComponentData& comp_data, const std::vector<std::shared_ptr<const AbstractShape>>& restricted_areas,
        const transform::Position& robot_position, const transform::Position& observe_position = "") const = 0;

    /**
     * @brief return whether this targetFeature is reachable by definition or not
     * @note An AntiTarget would not be reachable by definition in contrast to a default target, which is reachable.
     */
    [[nodiscard]] bool isReachable() const { return this->reachable; };

    /**
     * @brief returns if this feature is currently reached, if this feature is reachable by definition.
     * @param wm WorldModel
     * @param td TaskData
     * @param robot robot
     * @param time time point
     * @return if this feature is currently reached
     */
    [[nodiscard]] virtual bool isReached(const ComponentData& /*comp_data*/) const { return false; };

   protected:
    /**
     * @brief Construct a new Abstract Target Feature object
     *
     * @param shape shape of feature
     * @param reachable whether this feature is reachable by definition
     * @param weight weight of the feature
     */
    AbstractTargetFeature(std::shared_ptr<const AbstractShape> shape, bool reachable,
                          const DoubleComponentParam& weight = 1.0);

   private:
    /// @brief return whether this targetFeature is reachable by definition or not
    const bool reachable;
};

/**
 * @brief This class implements a base circular field obstacle
 * Circular field obstacle provide a 'calcForceVector' function that returns the circular fields force that is caused by
 * the obstacle. This force should then be applied on the robot.
 */
class AbstractObstacle : public AbstractFeature {
   public:
    /// @brief return that this feature is a circular field obstacle
    [[nodiscard]] constexpr FeatureBaseType getFeatureBaseType() const override { return FeatureBaseType::OBSTACLE; };

    [[nodiscard]] std::optional<std::pair<Eigen::Vector2d, Eigen::Vector2d>> getVecAndVelocity(
        const ComponentData& comp_data, const transform::Position& robot_position,
        const transform::Position& observe_position = "") const;

    [[nodiscard]] virtual std::optional<Eigen::Vector2d> getVelocityCommand(
        const ComponentData& comp_data, const Eigen::Vector2d& command_velocity, bool bypass_direction,
        double mean_target_distance, const transform::Position& robot_position,
        const transform::Position& observe_position = "") const;

    [[nodiscard]] virtual std::optional<RobotIdentifier> getRobot() const { return std::nullopt; }

    [[nodiscard]] double getCriticalDistance(const ComponentData& comp_data) const {
        return this->critical_distance.val(comp_data);
    };

    [[nodiscard]] double getInfluenceDistance(const ComponentData& comp_data) const {
        return this->influence_distance.val(comp_data);
    }

    [[nodiscard]] bool isRestrictedArea(const ComponentData& comp_data) const {
        return this->is_restricted_area.val(comp_data);
    }
    [[nodiscard]] bool breakRobotTowardsObstacle(const ComponentData& comp_data) const {
        return this->break_robot_towards_obstacle.val(comp_data);
    }
    [[nodiscard]] bool avoidCollision(const ComponentData& comp_data) const {
        return this->avoid_collision.val(comp_data);
    }

   protected:
    /**
     * @brief Construct a new Abstract C F Obstacle object
     *
     * @param shape shape of feature
     * @param weight weight of the feature
     * @param max_vel_x optional custom max velocity in x direction
     * @param max_vel_y optional custom max velocity in y direction
     */
    explicit AbstractObstacle(std::shared_ptr<const AbstractShape> shape, const DoubleComponentParam& weight = 1.0,
                              const std::optional<DoubleComponentParam>& critical_distance = std::nullopt,
                              const std::optional<DoubleComponentParam>& influence_distance = std::nullopt);

    DoubleComponentParam critical_distance;
    DoubleComponentParam influence_distance;

    BoolComponentParam is_restricted_area{true};
    BoolComponentParam break_robot_towards_obstacle{true};
    BoolComponentParam avoid_collision{true};

    // configuration methods
   public:
    void setCriticalDistance(DoubleComponentParam critical_distance) {
        this->critical_distance = std::move(critical_distance);
    }
    void setInfluenceDistance(DoubleComponentParam influence_distance) {
        this->influence_distance = std::move(influence_distance);
    }
    void setISRestrictedArea(BoolComponentParam is_restricted_area) {
        this->is_restricted_area = std::move(is_restricted_area);
    }
    void setBreakTowardsObstacle(BoolComponentParam break_robot_towards_obstacle) {
        this->break_robot_towards_obstacle = std::move(break_robot_towards_obstacle);
    }
    void setAvoidCollision(BoolComponentParam avoid_collision) { this->avoid_collision = std::move(avoid_collision); }
};

}  // namespace luhsoccer::robot_control