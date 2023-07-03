# Steps {#local_planner_steps}
`Steps` are the core component of [Skills](../skills.md), which are build from an ordered list of `Steps`. `Steps` calculate the [RobotCommand](../../../robot_interface/README.md) message that is sent to a specific robot in a control loop cycle. In every cycle the [LocalPlanner](../local_panner.md) request the current command message from the current `Step` of the current `Skill`. Along with the [RobotCommand](../../../robot_interface/README.md) message a state is returned by the skill, which signals whether the current `Step` should be "consulted" in the next loop cycle or the `LocalPlanner` can proceed to the next `Step`.

## Unified Methods
Steps implement two unified methods: `calcCommandMessage` and `getVisualizationMarkers`.
### calcCommandMessage
The `calcCommandMessage` method is the main method of skills and calculated the [RobotCommand](../../../robot_interface/README.md) that should be sent to specific robot in the current loop cycle. It is defined as follows:
```cpp
[[nodiscard]] std::pair<State, robot_interface::RobotCommand> calcCommandMessage(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const time::TimePoint time = time::TimePoint(0)) const;
```

### getVisualizationMarkers
`getVisualizationMarkers` should return an `std::vector` of [Markers](../../../marker_service/README.md) that should be visualized in this loop cycle. This is useful to visualize vector calculations especially in the `DriveStep`. It is defined as follows:
```cpp
[[nodiscard]] std::vector<marker::Marker> getVisualizationMarkers(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td,
        const RobotIdentifier& robot, const time::TimePoint time = time::TimePoint(0)) const;
```
> It is optional to override this function. By default it will return an empty array.


## Template
<details>
  <summary>template_step.hpp</summary>

```cpp
#pragma once

#include "local_planner/skills/skill_util.hpp"
#include "local_planner/skills/abstract_step.hpp"

namespace luhsoccer::local_planner {

class TemplateStep : public AbstractStep {
   public:
    TemplateStep(/*Component parameters here*/);

    //has to be overwritten
    [[nodiscard]] std::pair<State, robot_interface::RobotCommand> calcCommandMessage(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        time::TimePoint time = time::TimePoint(0)) const override;

    // optional to override
    [[nodiscard]] std::vector<marker::Marker> getVisualizationMarkers(
        const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& robot,
        const time::TimePoint time = time::TimePoint(0)) const override;

    private:
        /*Store Component parameters here*/
}; 

}  // namespace luhsoccer::local_planner
```
</details>

<details>
  <summary>template_step.cpp</summary>

```cpp
#include "local_planner_components/steps/template_step.hpp"

namespace luhsoccer::local_planner {

TemplateStep::TemplateStep(/*Component parameters here*/) : /*Init Component parameters here*/ {}

std::pair<AbstractStep::State, robot_interface::RobotCommand> TemplateStep::calcCommandMessage(
    const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td, const RobotIdentifier& /*robot*/,
    time::TimePoint time) const {
    //code here
}

std::vector<marker::Marker> TemplateStep::getVisualizationMarkers(const std::shared_ptr<const transform::WorldModel>& wm,
                                                               const TaskData& td, const RobotIdentifier& robot,
                                                               const time::TimePoint time) const {
    //code here
}
}  // namespace luhsoccer::local_planner
```
</details>
