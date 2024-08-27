#pragma once

#include <utility>

#include "robot_control/components/abstract_feature.hpp"

namespace luhsoccer::robot_control {

constexpr double DEFAULT_TRANSLATIONAL_TOLERANCE = 0.05;
class TargetFeature : public AbstractTargetFeature {
   public:
    [[nodiscard]] std::pair<Eigen::Vector2d, Eigen::Vector2d> calcArtificialDesiredVelocity(
        const ComponentData& comp_data, const std::vector<std::shared_ptr<const AbstractShape>>& restricted_areas,
        const transform::Position& robot_position, const transform::Position& observe_position = "") const override;

    [[nodiscard]] bool isReached(const ComponentData& comp_data) const override;

    [[nodiscard]] marker::Color getVisualizationColor(const ComponentData& /*comp_data*/) const override {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
        return RC_GREEN;
    };

   private:
    [[nodiscard]] Eigen::Vector2d getUnrestrictedGoal(
        const Eigen::Vector2d& goal_position, const ComponentData& comp_data,
        const std::vector<std::shared_ptr<const AbstractShape>>& restricted_areas,
        const transform::Position& observe_position = "", double margin = 0.0) const;

    static constexpr double ZERO_VELOCITY_TOLERANCE = 0.1;

    DoubleComponentParam translational_tolerance;
    BoolComponentParam ignore_velocity;
    BoolComponentParam velocity_zero_for_reach;
    DoubleComponentParam k_p;

    // configuration methods
   public:
    template <typename T>
    explicit TargetFeature(const T& shape,
                           DoubleComponentParam translational_tolerance = DEFAULT_TRANSLATIONAL_TOLERANCE,
                           const DoubleComponentParam& weight = 1.0, BoolComponentParam ignore_velocity = false,
                           BoolComponentParam velocity_zero_for_reach = false, DoubleComponentParam k_p = 1.0)
        : AbstractTargetFeature(std::make_shared<T>(shape), true, weight),
          translational_tolerance(std::move(translational_tolerance)),
          ignore_velocity(std::move(ignore_velocity)),
          velocity_zero_for_reach(std::move(velocity_zero_for_reach)),
          k_p{std::move(k_p)} {}

    void setTranslationalTolerance(DoubleComponentParam translational_tolerance) {
        this->translational_tolerance = std::move(translational_tolerance);
    }
    void setIgnoreVelocity(BoolComponentParam ignore_velocity) { this->ignore_velocity = std::move(ignore_velocity); }
    void setVelocityZeroForReach(BoolComponentParam velocity_zero_for_reach) {
        this->velocity_zero_for_reach = std::move(velocity_zero_for_reach);
    }
    void setKp(DoubleComponentParam k_p) { this->k_p = std::move(k_p); }
};

class BallTargetFeature : public TargetFeature {
   public:
    template <typename T>
    explicit BallTargetFeature(const T& shape, const DoubleComponentParam& weight = 1.0,
                               BoolComponentParam ignore_velocity = false, const DoubleComponentParam& k_p = 1.0)
        : TargetFeature(std::move(shape), 0.0, weight, ignore_velocity, false, k_p) {}

    [[nodiscard]] bool isReached(const ComponentData& comp_data) const override;
};

}  // namespace luhsoccer::robot_control