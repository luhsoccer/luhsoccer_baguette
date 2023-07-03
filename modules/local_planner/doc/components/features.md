# Features {#local_planner_features}
`Features` are used by the `DriveStep`. `Features` are all geometrical objects that influence the translational component of robot motion during the `DriveStep`. They can be split in two types: `TargetFeatures` and `CFObstacle`. All `Features` have a [Shape](shapes.md) and a weight (0-1). `Features` in genera have only one unified method which is `getVisualizationColor`. This method is used to color code the visualization marker from the `shape`.

## TargetFeatures
`TargetFeatures` describe geometrical objects the robot is actively attracted (or repelled) from. All `TargetFeatures` provide a method `calcArtificialDesiredVelocity` which returns a desired velocity vector to reach (or avoid) that specific feature. `TargetFeatures` can be generally reachable or generally not reachable. If the `TargetFeatures` is reachable, it should also override the `isReached` functions, which returns if the `Feature` is currently reached. What reached means is defined by the `TargetFeatures` itself.

### Template
<details>
  <summary>template_target_feature.hpp</summary>

```cpp
#pragma once

#include "local_planner/skills/abstract_feature.hpp"
#include "config/config_store.hpp"

namespace luhsoccer::local_planner {

class TemplateTargetFeature : public AbstractTargetFeature {
   public:
    TemplateTargetFeature(std::shared_ptr<const AbstractShape> shape, 
                  const DoubleComponentParam& weight = 1.0,
                  /* add ComponentsParams with a default value here */
                  const std::optional<DoubleComponentParam>& max_vel_x = std::nullopt,
                  const std::optional<DoubleComponentParam>& max_vel_y = std::nullopt
                  /* add optional ComponentsParams here */
                  );

    [[nodiscard]] Eigen::Vector2d calcArtificialDesiredVelocity(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const time::TimePoint& time = time::TimePoint(0),
        const ComponentPosition& observe_position = "") const override;

    [[nodiscard]] bool isReached(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                                 const RobotIdentifier& robot,
                                 const time::TimePoint& time = time::TimePoint(0)) const override;

    [[nodiscard]] marker::Color getVisualizationColor(const std::shared_ptr<const transform::WorldModel>& wm,
                                                      const TaskData& td, const RobotIdentifier& robot,
                                                      time::TimePoint time = time::TimePoint(0)) const override;

   private:
    /* store ComponentsParams here */
};

}  // namespace luhsoccer::local_planner
```
</details>

<details>
  <summary>template_target_feature.cpp</summary>

```cpp
#include "local_planner_components/steps/template_target_feature.hpp"

namespace luhsoccer::local_planner {
constexpr bool REACHABLE = true;
TemplateTargetFeature::TemplateTargetFeature(std::shared_ptr<const AbstractShape> shape, 
                  const DoubleComponentParam& weight,
                  /* add ComponentsParams with a default value here */
                  const std::optional<DoubleComponentParam>& max_vel_x,
                  const std::optional<DoubleComponentParam>& max_vel_y
                  /* add optional ComponentsParams here */)
                  : AbstractTargetFeature(std::move(shape), REACHABLE,  weight, max_vel_x, max_vel_y)
                  /*Init Component parameters here*/ 
                  {}

Eigen::Vector2d TemplateTargetFeature::calcArtificialDesiredVelocity(const std::shared_ptr<const transform::WorldModel>& wm,
                                                             const TaskData& td, const RobotIdentifier& robot,
                                                             const time::TimePoint& time,
                                                             const ComponentPosition& observe_position) const {
    // code here 
};

bool TemplateTargetFeature::isReached(const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
                              const RobotIdentifier& robot, const time::TimePoint& time) const {
    // code here
};

marker::Color TemplateTargetFeature::getVisualizationColor(const std::shared_ptr<const transform::WorldModel>& wm,
                                                      const TaskData& td, const RobotIdentifier& robot,
                                                      time::TimePoint time = time::TimePoint(0)) const{
    // code here
};

}  // namespace luhsoccer::local_planner
```
</details>

## CFObstacle
`CFObstacle` are obstacles that the robot tries to avoid but is not repelled from. A `CFObstacle` has no influence when the robot is near as long the robot moves away or does not move at all. The principles of `CFObstacle` features base on `Circular Fields`. They implement one unified method called `calcForceVector` which returns a force that should be applied on the robot in order to avoid this obstacle.

### Template
<details>
  <summary>template_cf_obstacle_feature.hpp</summary>

```cpp
#pragma once

#include "local_planner/skills/abstract_feature.hpp"
#include "config/config_store.hpp"

namespace luhsoccer::local_planner {

class TemplateCFObstacleFeature : public AbstractTargetFeature {
   public:
    TemplateCFObstacleFeature(std::shared_ptr<const AbstractShape> shape, 
                  const DoubleComponentParam& weight = 1.0,
                  /* add ComponentsParams with a default value here */
                  const std::optional<DoubleComponentParam>& max_vel_x = std::nullopt,
                  const std::optional<DoubleComponentParam>& max_vel_y = std::nullopt
                  /* add optional ComponentsParams here */
                  );

    [[nodiscard]] std::optional<std::pair<Eigen::Vector2d, Eigen::Vector2d>> calcForceVectorImpl(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, double max_target_distance,
        bool rotation_vector_upwards, const RobotIdentifier& robot, const time::TimePoint& time = time::TimePoint(0),
        const ComponentPosition& observe_position = "") const override;

    [[nodiscard]] marker::Color getVisualizationColor(const std::shared_ptr<const transform::WorldModel>& wm,
                                                      const TaskData& td, const RobotIdentifier& robot,
                                                      time::TimePoint time = time::TimePoint(0)) const override;
                                                      
    [[nodiscard]] marker::Marker getVisualizationMarker(const std::shared_ptr<const transform::WorldModel>& wm,
                                                        const TaskData& td, const RobotIdentifier& robot,
                                                        time::TimePoint time = time::TimePoint(0)) const override;

   private:
    /* store ComponentsParams here */
};

}  // namespace luhsoccer::local_planner
```
</details>

<details>
  <summary>template_cf_obstacle_feature.cpp</summary>

```cpp
#include "local_planner_components/features/template_cf_obstacle_feature.hpp"

namespace luhsoccer::local_planner {

TemplateCFObstacleFeature::TemplateCFObstacleFeature(std::shared_ptr<const AbstractShape> shape, 
                  const DoubleComponentParam& weight,
                  /* add ComponentsParams with a default value here */
                  const std::optional<DoubleComponentParam>& max_vel_x,
                  const std::optional<DoubleComponentParam>& max_vel_y
                  /* add optional ComponentsParams here */
                  )
    : AbstractCFObstacle(std::move(shape),  weight, max_vel_x, max_vel_y),
      /*Init Component parameters here*/
      {};

std::optional<std::pair<Eigen::Vector2d, Eigen::Vector2d>> TemplateCFObstacleFeature::calcForceVectorImpl(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, double max_target_distance,
    bool rotation_vector_upwards, const RobotIdentifier& robot, const time::TimePoint& time,
    const ComponentPosition& observe_position) const {
    //code here
};

marker::Color TemplateCFObstacleFeature::getVisualizationColor(const std::shared_ptr<const transform::WorldModel>& wm,
                                                      const TaskData& td, const RobotIdentifier& robot,
                                                      time::TimePoint time = time::TimePoint(0)) const{
    // code here
};

}  // namespace luhsoccer::local_planner
```
</details>