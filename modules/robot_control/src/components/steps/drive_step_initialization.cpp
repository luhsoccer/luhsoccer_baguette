
#include "robot_control/components/steps/drive_step.hpp"
#include "components/steps/drive_step_constraint.hpp"
#include "config/robot_control_config.hpp"

// constraints
#include "drive_step_constraints/stop_state_constraint.hpp"
#include "drive_step_constraints/avoid_other_robots.hpp"
#include "drive_step_constraints/defense_area.hpp"
#include "drive_step_constraints/boundaries.hpp"
#include "drive_step_constraints/enemy_kickoff.hpp"
#include "drive_step_constraints/leave_field.hpp"
#include "drive_step_constraints/ball_placement.hpp"

namespace luhsoccer::robot_control {

std::map<DriveStepConstraintNames, std::unique_ptr<DriveStepConstraint>> initConstraints() {
    std::map<DriveStepConstraintNames, std::unique_ptr<DriveStepConstraint>> map;
    map.emplace(DriveStepConstraintNames::STOP_STATE, std::make_unique<StopStateConstraint>());
    map.emplace(DriveStepConstraintNames::AVOID_OTHER_ROBOTS, std::make_unique<AvoidOtherRobotsConstraint>());
    map.emplace(DriveStepConstraintNames::DEFENSE_AREA, std::make_unique<DefenseAreaConstraint>());
    map.emplace(DriveStepConstraintNames::BOUNDARIES, std::make_unique<BoundariesConstraint>());
    map.emplace(DriveStepConstraintNames::ENEMY_KICKOFF, std::make_unique<EnemyKickOffConstraint>());
    map.emplace(DriveStepConstraintNames::LEAVE_FIELD, std::make_unique<LeaveFieldConstraint>());
    map.emplace(DriveStepConstraintNames::BALL_PLACEMENT, std::make_unique<BallPlacementConstraint>());
    return map;
};

DriveStep::DriveStep()
    : reach_condition(ReachCondition::ONE_OF_TARGETS),
      max_vel_x(robotControlConfig().robot_max_vel_x),
      max_vel_y(robotControlConfig().robot_max_vel_y),
      max_vel_theta(robotControlConfig().robot_max_vel_theta),
      constraints(initConstraints()) {}

DriveStep::DriveStep(DriveStep&&) noexcept = default;
DriveStep::~DriveStep() = default;

void DriveStep::setAvoidOtherRobots(const BoolComponentParam& avoid_other_robots) {
    this->constraints[DriveStepConstraintNames::AVOID_OTHER_ROBOTS]->setActive(avoid_other_robots);
};
void DriveStep::setAvoidDefenseArea(const BoolComponentParam& avoid_defense_area) {
    this->constraints[DriveStepConstraintNames::DEFENSE_AREA]->setActive(avoid_defense_area);
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
        case FeatureBaseType::OBSTACLE: {
            // try to cast base feature to derived obstacle feature
            std::shared_ptr<const AbstractObstacle> f = std::dynamic_pointer_cast<const AbstractObstacle>(feature);

            if (f) this->cf_features.push_back(std::move(f));

            break;
        }
        default:
            throw std::runtime_error("Unhandled feature base type in drive step!");
    }
}

}  // namespace luhsoccer::robot_control