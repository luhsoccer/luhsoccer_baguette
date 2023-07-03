#pragma once

#include <memory>
#include <vector>
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::transform {
class WorldModel;
}

namespace luhsoccer::local_planner {
struct TaskData;
class AbstractTargetFeature;
class AbstractCFObstacle;

class DriveStepConstraint {
   public:
    DriveStepConstraint() = default;
    DriveStepConstraint(const DriveStepConstraint&) = default;
    DriveStepConstraint(DriveStepConstraint&&) = default;
    DriveStepConstraint& operator=(const DriveStepConstraint&) = default;
    DriveStepConstraint& operator=(DriveStepConstraint&&) = default;
    virtual ~DriveStepConstraint() = default;

    [[nodiscard]] bool conditionMeet(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) const {
        return this->active.val(wm, td) && this->conditionMeetImpl(wm, td);
    }
    [[nodiscard]] std::vector<std::shared_ptr<const AbstractTargetFeature>> getAdditionalTargetFeatures() const {
        return target_features;
    }
    [[nodiscard]] std::vector<std::shared_ptr<const AbstractCFObstacle>> getAdditionalObstacleFeatures() const {
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
    [[nodiscard]] virtual std::vector<std::shared_ptr<const AbstractCFObstacle>> getAdditionalObstacleFeaturesImpl()
        const {
        return {};
    };
    [[nodiscard]] virtual bool conditionMeetImpl(const std::shared_ptr<const transform::WorldModel>& wm,
                                                 const TaskData& td) const = 0;
    BoolComponentParam active{true};
    std::vector<std::shared_ptr<const AbstractTargetFeature>> target_features;
    std::vector<std::shared_ptr<const AbstractCFObstacle>> obstacle_features;
};

}  // namespace luhsoccer::local_planner