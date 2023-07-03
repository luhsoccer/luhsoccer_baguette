# LocalPlanner Module {#local_planner}
The LocalPlanner Module provides classes to control the motion of the robot by providing specific tasks. Tasks consist of a skill and robot that should execute that skill. Task can also include skill specific data, i.e, the robot that should be marked by out robot, or a point where the robot should drive to.

> More information about skills can be found in the [skills module](../skills/README.md)

## Interface
At initialization the `LocalPlannerModule` requires the main [WorldModel](../transform/doc/world_model.md) from which it will create a [LocalPlanner](doc/local_panner.md) for every possible Robot (16 by default). Furthermore, the [RobotInterface](../robot_interface/README.md) is required. It is passed to the individual planners, so they can sent the robot commands through it.

The `LocalPlannerModule` provides two main methods for other modules, which both are thread safe: `setTask` and `cancelTask`. `setTask` sets a new Task for a specific robot. It requires a ``Skill`` pointer (see [skills module](../skills/README.md) on how to obtain it) and a [TaskData](doc/task_data.md) object. Among other things, the TaskData object provides information about which robot should execute the task. Based on that the skill and [TaskData](doc/task_data.md) is forwarded to the corresponding module. The method returns feedback whether the task could be set or not. Reasons the task set is unsuccessful include the following:
- The planner is executing another skill -> cancel the previous task
- The TaskData is invalid (i.e. required points or robot are missing).
- The Skill is a ``nullptr``.
- The provided robot identifier is an identifier for an enemy robot -> we cannot control enemy robots...
- The requested robot is not on the field

The `cancelTask` method cancels the current task of a given robot. It returns `false` if the requested planner currently is not executing any task.

## Structure
As stated before, the `LocalPlannerModule` creates a [LocalPlanner](doc/local_panner.md) for every robot, which are responsible to control one specific robot each. All Planner create one real [Agent](doc/agent.md) which computes the current command messages for the robot. The Planner feeds the real [Agent](doc/agent.md) with new tasks and cancels them if needed. The [Agent](doc/agent.md) then executes the [skill](doc/skills.md) that is provided. 


# Topics:

1. LocalPlanner @subpage local_planner_local_planner `coming soon`
2. Agent @subpage local_planner_agents
3. Skills @subpage local_planner_skills
4. TaskData @subpage local_planner_task_data
5. Components @subpage local_planner_components