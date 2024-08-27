#pragma once

#include <memory>
#include <utility>

#include "robot_control/components/abstract_shape.hpp"
#include "robot_control/components/component_util.hpp"

namespace luhsoccer::robot_control {

class ComposeShape : public AbstractShape {
   public:
    explicit ComposeShape(std::vector<std::shared_ptr<const AbstractShape>> shapes = {}) : shapes(std::move(shapes)){};

    [[nodiscard]] VectorWithVelocityStamped getTransformToClosestPoint(
        const ComponentData& comp_data, const transform::Position& robot_position,
        const transform::Position& observe_position = "") const override;

    [[nodiscard]] std::optional<Eigen::Vector2d> getCenter(
        const ComponentData& comp_data, const transform::Position& observe_position = "") const override;

    [[nodiscard]] bool hasArea(const ComponentData& comp_data) const override;

    [[nodiscard]] double getAreaSize(const ComponentData& comp_data) const override;

    [[nodiscard]] std::vector<Eigen::Vector2d> getPointsOnBrim(const ComponentData& comp_data,
                                                               double spacing = DEFAULT_BRIM_SPACING,
                                                               const transform::Position& observe_position = "",
                                                               double margin = 0.0) const override;
    [[nodiscard]] bool isPointInArea(const Eigen::Vector2d& point, const ComponentData& comp_data,
                                     const transform::Position& observe_position = "") const override;

    template <typename T>
    void addShape(T&& shape) {
        static_assert(std::is_base_of<AbstractShape, T>::value);
        this->shapes.push_back(std::make_shared<T>(std::forward<T>(shape)));
    }

   private:
    std::vector<std::shared_ptr<const AbstractShape>> shapes;
};

}  // namespace luhsoccer::robot_control