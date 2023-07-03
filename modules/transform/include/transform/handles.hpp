#pragma once
#include <utility>

#include "common_types.hpp"
#include "transform/transform.hpp"
#include "robot_identifier.hpp"

namespace luhsoccer::transform {

class RobotHandle {
   private:
    std::weak_ptr<const WorldModel> wm;
    RobotIdentifier id;

   public:
    RobotHandle(const RobotIdentifier& id, const std::shared_ptr<const WorldModel>& wm) : wm(wm), id(id){};

    [[nodiscard]] transform::Position getPosition() const { return {this->id.getFrame()}; }
    [[nodiscard]] std::optional<RobotData> getRobotData(const time::TimePoint& time = time::TimePoint(0)) const {
        if (const auto wm_ptr = this->wm.lock()) return wm_ptr->getRobotData(this->id, time);
        return std::nullopt;
    }
    [[nodiscard]] std::optional<AllyRobotData> getAllyRobotData(
        const time::TimePoint& time = time::TimePoint(0)) const {
        if (const auto wm_ptr = this->wm.lock()) return wm_ptr->getAllyRobotData(this->id, time);
        return std::nullopt;
    }
    [[nodiscard]] RobotIdentifier getID() const { return this->id; }
    [[nodiscard]] std::weak_ptr<const WorldModel> getWorldModel() const { return this->wm; }

    [[nodiscard]] bool isLinkedWorldModel(const std::shared_ptr<const WorldModel>& wm) const {
        return !this->wm.expired() && this->wm.lock() == wm;
    }
    [[nodiscard]] bool isWorldModelValid() const { return this->wm.expired(); }

    [[nodiscard]] std::optional<Eigen::Affine2d> getAffine() const {
        if (const auto wm_ptr = this->getWorldModel().lock()) {
            return this->getPosition().getCurrentPosition(wm_ptr);
        }
        return std::nullopt;
    }

    [[nodiscard]] constexpr bool isAlly() const noexcept { return this->id.isAlly(); }
    [[nodiscard]] constexpr bool isEnemy() const noexcept { return this->id.isEnemy(); }
    [[nodiscard]] constexpr Team getTeam() const noexcept { return this->id.getTeam(); }
};

class WritableRobotHandle : public RobotHandle {
   private:
    std::weak_ptr<WorldModel> wm;

   public:
    WritableRobotHandle(const RobotIdentifier& id, const std::shared_ptr<WorldModel>& wm)
        : RobotHandle(id, wm), wm(wm){};
};

}  // namespace luhsoccer::transform
