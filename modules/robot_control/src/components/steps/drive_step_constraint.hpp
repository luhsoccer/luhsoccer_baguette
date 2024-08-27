#pragma once

#include <memory>
#include <vector>
#include "robot_control/components/component_util.hpp"

namespace luhsoccer::robot_control {
class AbstractTargetFeature;
class AbstractObstacle;
struct ComponentData;

class DriveStepConstraint {
   public:
    DriveStepConstraint() = default;
    DriveStepConstraint(const DriveStepConstraint&) = default;
    DriveStepConstraint(DriveStepConstraint&&) = default;
    DriveStepConstraint& operator=(const DriveStepConstraint&) = default;
    DriveStepConstraint& operator=(DriveStepConstraint&&) = default;
    virtual ~DriveStepConstraint() = default;

    [[nodiscard]] bool conditionMet(const ComponentData& comp_data) const {
        return this->active.val(comp_data) && this->conditionMeetImpl(comp_data);
    }
    [[nodiscard]] std::vector<std::shared_ptr<const AbstractTargetFeature>> getAdditionalTargetFeatures() const {
        return target_features;
    }
    [[nodiscard]] std::vector<std::shared_ptr<const AbstractObstacle>> getAdditionalObstacleFeatures() const {
        return obstacle_features;
    }

    void setActive(BoolComponentParam active) { this->active = std::move(active); }

   protected:
    void initFeatures() {
        this->target_features = this->getAdditionalTargetFeaturesImpl();
        this->obstacle_features = this->getAdditionalObstacleFeaturesImpl();
    }

   private:
    [[nodiscard]] virtual std::vector<std::shared_ptr<const AbstractTargetFeature>> getAdditionalTargetFeaturesImpl()
        const {
        return {};
    };
    [[nodiscard]] virtual std::vector<std::shared_ptr<const AbstractObstacle>> getAdditionalObstacleFeaturesImpl()
        const {
        return {};
    };
    [[nodiscard]] virtual bool conditionMeetImpl(const ComponentData& comp_data) const = 0;

    BoolComponentParam active{true};
    std::vector<std::shared_ptr<const AbstractTargetFeature>> target_features;
    std::vector<std::shared_ptr<const AbstractObstacle>> obstacle_features;
};

}  // namespace luhsoccer::robot_control