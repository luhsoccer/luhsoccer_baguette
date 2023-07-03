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

#include "local_planner/skills/skill_util.hpp"
#include "marker_service/marker.hpp"
#include "local_planner/skills/abstract_component.hpp"
#include "config_provider/config_store_main.hpp"
namespace luhsoccer {

class RobotIdentifier;

namespace local_planner {
class AbstractShape;

/// @brief enum of all subtypes of features
enum class FeatureBaseType { TARGET, CF_OBSTACLE };

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
    [[nodiscard]] double getWeight(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) const {
        return this->weight.val(wm, td);
    };

    /**
     * @brief Get the Shape
     *
     * @return const std::shared_ptr<const AbstractShape>& shape of feature
     */
    [[nodiscard]] const std::shared_ptr<const AbstractShape>& getShape() const { return this->shape; }

    /**
     * @brief Get the current visualization marker of this feature
     *
     * @param wm WorldModel
     * @param td TaskData
     * @param robot robot
     * @param time time point
     * @return Marker current marker of this feature
     */
    [[nodiscard]] virtual std::vector<Marker> getVisualizationMarker(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        time::TimePoint time = time::TimePoint(0)) const;

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
    [[nodiscard]] virtual marker::Color getVisualizationColor(const std::shared_ptr<const transform::WorldModel>& wm,
                                                              const TaskData& td, const RobotIdentifier& robot,
                                                              time::TimePoint time = time::TimePoint(0)) const = 0;

   protected:
    /**
     * @brief Construct a new Abstract Feature object
     *
     * @param shape shape of feature
     * @param weight weight of the feature
     * @param max_vel_x optional custom max velocity in x direction
     * @param max_vel_y optional custom max velocity in y direction
     */
    AbstractFeature(std::shared_ptr<const AbstractShape> shape, DoubleComponentParam weight = 1.0,
                    std::optional<DoubleComponentParam> max_vel_x = std::nullopt,
                    std::optional<DoubleComponentParam> max_vel_y = std::nullopt)
        : shape(std::move(shape)),
          weight(std::move(weight)),
          max_vel_x(localPlannerConfig().robot_vel_max_x),
          max_vel_y(localPlannerConfig().robot_vel_max_y) {
        if (max_vel_x) {
            this->max_vel_x = *max_vel_x;
        }
        if (max_vel_y) {
            this->max_vel_y = *max_vel_y;
        }
    };

    /**
     * @brief limit (clamp) the velocity vector to the maximum x and y direction in both dimensions separately
     * @note the direction of the incoming vector could change when the limit of one direction is reached
     * @param wm WorldModel
     * @param td TaskData
     * @param vec velocity vector
     * @return Eigen::Vector2d limited velocity Vector
     */
    [[nodiscard]] Eigen::Vector2d limitVelocityVector(const std::shared_ptr<const transform::WorldModel>& wm,
                                                      const TaskData& td, const Eigen::Vector2d& vec) const {
        // vec.x() *= std::min(1.0, max_vel_x.val(wm, td) / std::abs(vec.x()));
        // vec.y() *= std::min(1.0, max_vel_y.val(wm, td) / std::abs(vec.y()));
        return vec * std::min(1.0, max_vel_x.val(wm, td) / vec.norm());
    }

    /// @brief shape of this feature
    std::shared_ptr<const AbstractShape> shape;
    /// @brief the weight of this feature. The weight is a multiplier for the commands of this feature.
    DoubleComponentParam weight;
    /// @brief maximum absolute velocity in x (forward) direction [m/s].
    DoubleComponentParam max_vel_x;
    /// @brief maximum absolute velocity in y (sideways) direction [m/s].
    DoubleComponentParam max_vel_y;
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
    [[nodiscard]] virtual Eigen::Vector2d calcArtificialDesiredVelocity(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const time::TimePoint& time = time::TimePoint(0), const ComponentPosition& observe_position = "") const = 0;

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
    [[nodiscard]] virtual bool isReached(const std::shared_ptr<const transform::WorldModel>& /*wm*/,
                                         const TaskData& /*td*/, const RobotIdentifier& /*robot*/,
                                         const time::TimePoint& /*time*/ = time::TimePoint(0)) const {
        return false;
    };

   protected:
    /**
     * @brief Construct a new Abstract Target Feature object
     *
     * @param shape shape of feature
     * @param reachable whether this feature is reachable by definition
     * @param weight weight of the feature
     * @param max_vel_x optional custom max velocity in x direction
     * @param max_vel_y optional custom max velocity in y direction
     */
    AbstractTargetFeature(std::shared_ptr<const AbstractShape> shape, bool reachable,
                          const DoubleComponentParam& weight = 1.0,
                          const std::optional<DoubleComponentParam>& max_vel_x = std::nullopt,
                          const std::optional<DoubleComponentParam>& max_vel_y = std::nullopt)
        : AbstractFeature(std::move(shape), weight, max_vel_x, max_vel_y), reachable(reachable){};

   private:
    /// @brief return whether this targetFeature is reachable by definition or not
    const bool reachable;
};

/**
 * @brief This class implements a base circular field obstacle
 * Circular field obstacle provide a 'calcForceVector' function that returns the circular fields force that is caused by
 * the obstacle. This force should then be applied on the robot.
 */
class AbstractCFObstacle : public AbstractFeature {
   public:
    /// @brief return that this feature is a circular field obstacle
    [[nodiscard]] constexpr FeatureBaseType getFeatureBaseType() const override {
        return FeatureBaseType::CF_OBSTACLE;
    };

    [[nodiscard]] std::optional<std::pair<Eigen::Vector2d, Eigen::Vector2d>> getVecAndVelocity(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const time::TimePoint& time = time::TimePoint(0), const ComponentPosition& observe_position = "") const;

    [[nodiscard]] virtual std::optional<RobotIdentifier> getRobot() const { return std::nullopt; }

    [[nodiscard]] double getCriticalDistance(const std::shared_ptr<const transform::WorldModel>& wm,
                                             const TaskData& td) const {
        return this->critical_distance.val(wm, td);
    }

    [[nodiscard]] double getInfluenceDistance(const std::shared_ptr<const transform::WorldModel>& wm,
                                              const TaskData& td) const {
        return this->influence_distance.val(wm, td);
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
    AbstractCFObstacle(std::shared_ptr<const AbstractShape> shape, const DoubleComponentParam& weight = 1.0,
                       const std::optional<DoubleComponentParam>& max_vel_x = std::nullopt,
                       const std::optional<DoubleComponentParam>& max_vel_y = std::nullopt,
                       const std::optional<DoubleComponentParam>& critical_distance = std::nullopt,
                       const std::optional<DoubleComponentParam>& influence_distance = std::nullopt)
        : AbstractFeature(std::move(shape), weight, max_vel_x, max_vel_y),
          critical_distance(critical_distance.has_value() ? critical_distance.value()
                                                          : localPlannerConfig().obstacle_collision_distance),
          influence_distance(influence_distance.has_value()
                                 ? influence_distance.value()
                                 : localPlannerConfig().feature_robot_obstacle_influence_distance){};

    const DoubleComponentParam critical_distance;
    const DoubleComponentParam influence_distance;
};
using RotationVectorCallback = std::function<std::vector<bool>(
    const std::vector<std::shared_ptr<const AbstractCFObstacle>>& feature, const Eigen::Vector2d& goal_vec,
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
    const time::TimePoint time)>;

template <typename... Ts>
inline std::vector<std::shared_ptr<const AbstractFeature>> featureList(Ts&&... features) {
    std::vector<std::shared_ptr<const AbstractFeature>> feature_list{
        std::make_shared<Ts...>(std::forward<Ts...>(features...))};

    return feature_list;
}

}  // namespace local_planner
}  // namespace luhsoccer