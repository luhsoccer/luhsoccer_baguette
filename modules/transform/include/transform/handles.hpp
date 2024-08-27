#pragma once
#include <utility>

#include "core/common_types.hpp"
#include "transform/transform.hpp"
#include "core/robot_identifier.hpp"

namespace luhsoccer::transform {

class RobotHandle {
   private:
    std::weak_ptr<const WorldModel> wm;
    RobotIdentifier id;

   public:
    RobotHandle(const RobotIdentifier& id, const std::shared_ptr<const WorldModel>& wm) : wm(wm), id(id){};

    [[nodiscard]] transform::Position getPosition() const { return {this->id.getFrame()}; }

    [[nodiscard]] std::optional<Eigen::Vector2d> getPosVec() const {
        auto pos = this->getPosAndRotVec();
        if (!pos.has_value()) return std::nullopt;
        return pos->head<2>();
    }

    [[nodiscard]] Eigen::Vector2d getPosVecOr(const Eigen::Vector2d& default_val) const {
        auto pos = this->getPosAndRotVec();
        if (!pos.has_value()) return default_val;
        return pos->head<2>();
    }

    /**
     * @brief Get the Position and Rotation of the Robot as a vector
     * or the given default value if the pos/Rot is not present
     *
     * @param default_val The Default value that will be returned if no value is present
     * @return Eigen::Vector3d The Position (x, y) and rotation (z) (radians) of the robot
     */
    [[nodiscard]] Eigen::Vector3d getPosAndRotVecOr(const Eigen::Vector3d& default_val) const {
        auto pos = this->getPosAndRotVec();
        if (!pos.has_value()) return default_val;
        return *pos;
    }

    /**
     * @brief Get the Position and Rotation of the Robot as a vector
     * or std::nullopt if the pos/Rot is not present
     *
     * @return Eigen::Vector3d The Position (x, y) and rotation (z) (radians) of the robot
     */
    [[nodiscard]] std::optional<Eigen::Vector3d> getPosAndRotVec() const {
        auto pos = this->getAffine();
        if (!pos.has_value()) return std::nullopt;

        // Combine the 2d position and the angle into a 3d vector
        Eigen::Vector3d result;
        result << pos->translation(), Eigen::Rotation2Dd(pos->rotation()).angle();
        return result;
    }

    [[nodiscard]] std::optional<Eigen::Vector3d> getVelocity() {
        const auto tf_pos = this->getPosition();
        const auto world_model = this->getWorldModel().lock();
        if (world_model == nullptr) return std::nullopt;

        auto vel = tf_pos.getVelocity(world_model, world_model->getGlobalFrame(), world_model->getGlobalFrame());
        return vel;
    }

    [[nodiscard]] Eigen::Vector3d getVelocityOr(const Eigen::Vector3d& default_value) {
        auto vel = this->getVelocity();
        if (!vel.has_value()) return default_value;
        return *vel;
    }

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
    [[nodiscard]] bool isWorldModelValid() const { return !this->wm.expired(); }

    [[nodiscard]] std::optional<Eigen::Affine2d> getAffine() const {
        if (const auto wm_ptr = this->getWorldModel().lock()) {
            return this->getPosition().getCurrentPosition(wm_ptr);
        }
        return std::nullopt;
    }

    [[nodiscard]] constexpr bool isAlly() const noexcept { return this->id.isAlly(); }
    [[nodiscard]] constexpr bool isEnemy() const noexcept { return this->id.isEnemy(); }
    [[nodiscard]] constexpr Team getTeam() const noexcept { return this->id.getTeam(); }
    [[nodiscard]] std::string getFrame() const noexcept { return this->id.getFrame(); }
};

class WritableRobotHandle : public RobotHandle {
   private:
    std::weak_ptr<WorldModel> wm;

   public:
    WritableRobotHandle(const RobotIdentifier& id, const std::shared_ptr<WorldModel>& wm)
        : RobotHandle(id, wm), wm(wm){};
};

}  // namespace luhsoccer::transform
