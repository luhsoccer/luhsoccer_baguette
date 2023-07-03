

#pragma once
#define DRIVE_STEP

#include <utility>

#include "local_planner/skills/skill_util.hpp"
#include "local_planner/skills/abstract_step.hpp"
#include "local_planner/skills/abstract_feature.hpp"
#include "local_planner/skills/abstract_rotation_control.hpp"
#include "local_planner/robot_dynamic.hpp"
#include "local_planner_components/shapes/point_shape.hpp"
#include "local_planner_components/steps/drive_step_constraint.hpp"
namespace luhsoccer::local_planner {

enum class DriveStepConstraintNames {
    STOP_STATE,
    DEFENSE_AREA,
    AVOID_OTHER_ROBOTS,
    KICKOFF_ENEMY,
    KICKOFF_ALLY,
    BALL_PLACEMENT,
};

class DriveStep : public AbstractStep {
   public:
    enum class ReachCondition { NEVER, ONE_OF_TARGETS, ALL_TARGETS };

    [[nodiscard]] std::pair<StepState, robot_interface::RobotCommand> calcCommandMessage(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const std::shared_ptr<AvoidanceManager>& am, const time::TimePoint time = time::TimePoint(0)) const override;

    [[nodiscard]] std::vector<Marker> getVisualizationMarkers(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const time::TimePoint time = time::TimePoint(0)) const override;

   private:
    [[nodiscard]] bool isReachedTranslational(const std::shared_ptr<const transform::WorldModel>& wm,
                                              const TaskData& td, const RobotIdentifier& robot,
                                              const time::TimePoint& time) const;

    [[nodiscard]] double calcTargetRelaxationFactors(const std::shared_ptr<const transform::WorldModel>& wm,
                                                     const TaskData& td,
                                                     const std::optional<Eigen::Vector2d>& min_obstacle_vec,
                                                     const Eigen::Vector2d& max_target_vec) const;

    [[nodiscard]] std::optional<robot_interface::RobotCommand> calcRobotCommandFromForce(
        const Eigen::Vector3d& total_force, const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
        const RobotIdentifier& robot, const time::TimePoint time,
        const std::vector<std::pair<Eigen::Vector2d, std::optional<bool>>>& critical_obstacle_vectors) const;

    void initAvoidOtherRobotFeatures();

    [[nodiscard]] robot_interface::RobotCommand getStopCommand() const;

    std::vector<std::shared_ptr<const AbstractTargetFeature>> target_features;
    std::vector<std::shared_ptr<const AbstractCFObstacle>> cf_features;

    std::shared_ptr<const AbstractRotationControl> rotation_control;

    ReachCondition reach_condition;
    DoubleComponentParam feature_target_k_v;
    DoubleComponentParam max_vel_x;
    DoubleComponentParam max_vel_y;
    DoubleComponentParam max_vel_theta;
    DoubleComponentParam step_drive_predict_duration_ms;
    RobotDynamic robot_dynamic;
    BoolComponentParam cancel_condition{false};
    BoolComponentParam ignore_rotation{false};
    std::map<DriveStepConstraintNames, std::unique_ptr<DriveStepConstraint>> constraints;

    //-------------------------Skill configurator methods-----------------------
   public:
    DriveStep(const std::vector<std::shared_ptr<const AbstractFeature>>& features,
              std::shared_ptr<const AbstractRotationControl> rotation_control, const ReachCondition& reach_condition,
              const BoolComponentParam& avoid_other_robots = true,
              std::optional<DoubleComponentParam> feature_target_k_v = std::nullopt,
              std::optional<DoubleComponentParam> max_vel_x = std::nullopt,
              std::optional<DoubleComponentParam> max_vel_y = std::nullopt,
              std::optional<DoubleComponentParam> max_vel_theta = std::nullopt,
              std::optional<DoubleComponentParam> step_drive_predict_duration_ms = std::nullopt);

    explicit DriveStep();

    template <typename T>
    void addFeature(T&& feature) {
        static_assert(std::is_base_of<AbstractFeature, T>::value);
        addFeatureImpl(std::make_shared<T>(std::forward<T>(feature)));
    }

    void setAvoidOtherRobots(const BoolComponentParam& avoid_other_robots);
    void setAvoidDefenseArea(const BoolComponentParam& avoid_defense_area);
    void activateConstraint(const DriveStepConstraintNames& name, const BoolComponentParam& active);
    void setObeyStopState(const BoolComponentParam& obey_stop_state);

    void setIgnoreRotation(const BoolComponentParam& ignore_rotation) { this->ignore_rotation = ignore_rotation; }

    template <typename T>
    void setRotationControl(T&& rotation_control) {
        static_assert(std::is_base_of<AbstractRotationControl, T>::value);
        this->rotation_control = std::make_shared<T>(std::forward<T>(rotation_control));
    }

    void setReachCondition(const ReachCondition& reach_condition) { this->reach_condition = reach_condition; }

    void setFeatureTargetKV(DoubleComponentParam feature_target_k_v) {
        this->feature_target_k_v = std::move(feature_target_k_v);
    }
    void setMaxVelX(DoubleComponentParam max_vel_x) { this->max_vel_x = std::move(max_vel_x); }
    void setMaxVelY(DoubleComponentParam max_vel_y) { this->max_vel_y = std::move(max_vel_y); }
    void setMaxVelT(DoubleComponentParam max_vel_theta) { this->max_vel_theta = std::move(max_vel_theta); }
    void setPredictDuration(DoubleComponentParam predict_duration_ms) {
        this->step_drive_predict_duration_ms = std::move(predict_duration_ms);
    }

    void setCancelCondition(BoolComponentParam cancel_condition) {
        this->cancel_condition = std::move(cancel_condition);
    }

   private:
    void addFeatureImpl(const std::shared_ptr<const AbstractFeature>& feature);
};

}  // namespace luhsoccer::local_planner