

#pragma once

#include <memory>
#include <utility>

#include "local_planner/skills/abstract_shape.hpp"
#include "local_planner/skills/skill_util.hpp"

namespace luhsoccer::local_planner {

class ComposeShape : public AbstractShape {
   public:
    explicit ComposeShape(std::vector<std::shared_ptr<const AbstractShape>> shapes = {}) : shapes(std::move(shapes)){};

    [[nodiscard]] VectorWithVelocityStamped getTransformToClosestPoint(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
        const ComponentPosition& reference_position, time::TimePoint time = time::TimePoint(0),
        const ComponentPosition& observe_position = {""}) const override;

    [[nodiscard]] std::vector<Marker> getVisualizationMarker(const std::shared_ptr<const transform::WorldModel>& wm,
                                                             const TaskData& td,
                                                             time::TimePoint time = time::TimePoint(0)) const override;

    [[nodiscard]] std::optional<Eigen::Vector2d> getCenter(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
        time::TimePoint time = time::TimePoint(0), const ComponentPosition& observe_position = {""}) const override;

    template <typename T>
    void addShape(T&& shape) {
        static_assert(std::is_base_of<AbstractShape, T>::value);
        this->shapes.push_back(std::make_shared<T>(std::forward<T>(shape)));
    }

   private:
    std::vector<std::shared_ptr<const AbstractShape>> shapes;
};

}  // namespace luhsoccer::local_planner