# TaskData {#local_planner_task_data}
The `TaskData`-Class stores information regarding a specific task. Along with the robot that should execute the task, the `TaskData`-class holds skill specific Information, i.e., the robot that should be marked or a position that should be reached. The [skills](skills.md) defines which data is required and the documentation of the [skill](skills.md) should state what the data is used for in the skill.

> Note that `TaskData` Objects are not multi threading safe (even though it might look like they are, as only a `const` references are used) and therefore should always be copied if used in different threads.
## Creating a `TaskData`-Object
Here is an example on how to create a `TaskData`-Object for a skill that requires one robot and two positions:
```cpp
TaskData td(our_robot);
td.related_robots = {enemy_robot};
td.required_positions = {transform::Position("", 3.0, 2.0), transform::Position("", 3.0, 2.0)};
```

## Checking if the task_data is valid
If a `TaskData`-Object is valid, can be verified using the [skill](skills.md) pointer. Valid with respect to a skill means that the number of positions and robots etc. in the TaskData is equal to the required data by the specific [skill](skills.md).
```cpp
const Skill* skill;
bool valid = skill->taskDataValid(td);
```
Only valid `TaskData` lead to a successful task execution.
> Note that the `TaskData`-Object is also invalid if too much data is provided.


## Cookie Jar
The cookie jar is used by [components](components/components.md) to store data for the next control loop. The principle stems from cookies in a browser, thus this name was chosen. It is not necessary to access the cookie jar manually. It would not be possible anyways since it is private. More information about cookies can be found in the [components](components/components.md).