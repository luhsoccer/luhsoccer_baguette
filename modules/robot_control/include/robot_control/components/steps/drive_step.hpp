#pragma once

#include "robot_control/components/abstract_feature.hpp"
#include "robot_control/components/abstract_step.hpp"
#include "robot_control/components/component_util.hpp"
#include "robot_control/components/abstract_rotation_control.hpp"

#include <memory>

namespace luhsoccer::robot_control {

class AbstractRotationControl;
class DriveStepConstraint;

enum class DriveStepConstraintNames {
    STOP_STATE,
    AVOID_OTHER_ROBOTS,
    DEFENSE_AREA,
    BOUNDARIES,
    ENEMY_KICKOFF,
    LEAVE_FIELD,
    BALL_PLACEMENT
};

class DriveStep : public AbstractStep {
   public:
    enum class ReachCondition { NEVER, ONE_OF_TARGETS, ALL_TARGETS };

    [[nodiscard]] std::pair<StepState, robot_interface::RobotCommand> calcCommandMessage(
        const ComponentData& comp_data) const override;

   private:
    [[nodiscard]] bool isReachedTranslational(const ComponentData& comp_data) const;

    [[nodiscard]] static robot_interface::RobotCommand getStopCommand();

    void displayPlots(const ComponentData& comp_data, const Eigen::Vector3d& robot_velocity,
                      const Eigen::Vector3d& cmd_velocity) const;

    std::vector<std::shared_ptr<const AbstractTargetFeature>> target_features;
    std::vector<std::shared_ptr<const AbstractObstacle>> cf_features;

    std::shared_ptr<const AbstractRotationControl> rotation_control;

    ReachCondition reach_condition;
    DoubleComponentParam max_vel_x;
    DoubleComponentParam max_vel_y;
    DoubleComponentParam max_vel_theta;

    BoolComponentParam cancel_condition{false};
    BoolComponentParam ignore_rotation{false};
    std::map<DriveStepConstraintNames, std::unique_ptr<DriveStepConstraint>> constraints;

    //-------------------------Skill configurator methods-----------------------
   public:
    explicit DriveStep();
    DriveStep& operator=(const DriveStep&) = delete;
    DriveStep& operator=(DriveStep&&) = delete;
    DriveStep(const DriveStep&) = delete;
    DriveStep(DriveStep&&) noexcept;
    ~DriveStep() override;

    template <typename T>
    void addFeature(T&& feature) {
        static_assert(std::is_base_of<AbstractFeature, T>::value);
        addFeatureImpl(std::make_shared<T>(std::forward<T>(feature)));
    }

    void setAvoidOtherRobots(const BoolComponentParam& avoid_other_robots);
    void setAvoidDefenseArea(const BoolComponentParam& avoid_defense_area);
    void activateConstraint(const DriveStepConstraintNames& name, const BoolComponentParam& active);

    void setIgnoreRotation(const BoolComponentParam& ignore_rotation) { this->ignore_rotation = ignore_rotation; }

    template <typename T>
    void setRotationControl(T&& rotation_control) {
        static_assert(std::is_base_of<AbstractRotationControl, T>::value);
        this->rotation_control = std::make_shared<T>(std::forward<T>(rotation_control));
    }

    void setReachCondition(const ReachCondition& reach_condition) { this->reach_condition = reach_condition; }

    void setMaxVelX(DoubleComponentParam max_vel_x) { this->max_vel_x = std::move(max_vel_x); }
    void setMaxVelY(DoubleComponentParam max_vel_y) { this->max_vel_y = std::move(max_vel_y); }
    void setMaxVelT(DoubleComponentParam max_vel_theta) { this->max_vel_theta = std::move(max_vel_theta); }

    void setCancelCondition(BoolComponentParam cancel_condition) {
        this->cancel_condition = std::move(cancel_condition);
    }

   private:
    void addFeatureImpl(const std::shared_ptr<const AbstractFeature>& feature);
};
}  // namespace luhsoccer::robot_control