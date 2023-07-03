# Skills Module {#skills}

## Define a new skill

### Create a new Skill

To create a new skill, just use the script called `create_skill.sh` in the skills module like so:

```bash
./create_skill.sh SkillName
```

Replace `SkillName` with the name of your new skill. After the execution of the script you will find a new file named like your skill in the `src/bod_skill_book` folder. You only need to edit this file to implement your new skill.

### Define the Input Arguments

After creating the skill, you need to define which input arguments (positions,robots,ints,doubles etc.) are required to start the skill.
Do this in the constructor of the SkillBuilder by providing a list of descriptive names of your input arguments. An example would look like this:

```cpp
AVeryCoolSkillBuild::AVeryCoolSkillBuild()
        : SkillBuilder(
              name: "AVeryCoolSkill",
              related_robot: {"EnemyRobot","MyAllyRobot2"},        //
              required_point: {"TargetPosiiton"},                    //
              required_double: {"AVeryCoolDoubleThatIUseForSomeThingUseful"},                    //
              required_int: {},                    //
              required_bool: {},  //
              required_string: {}){};
```

This Skill named `AVeryCoolSkill` would require two robots, one position and one double.

### Create new Step

Now you can define a new `Step`. Do this by including a step at the top of the file. Here is an example:

```cpp
#include "local_planner_components/steps/drive_step.hpp"
```

After this you can create the step in the `buildImpl` function:

```cpp
DriveStep new_step;
```

> Pay attention to the constructor of the `Step`. Some `Steps` required to define every property in the constructor, and some are configurable by calling member functions after the creation.

### Add the a new Skill

Next up you need to the `Step` to your skill. Do this by using the `addStep` function:

```cpp
addStep(std::move(new_step));
```

> In the case of the `DriveStep` the `addStep` function needs to be called after the configuration of the step was done.

### Configure the DriveStep

The `DriveStep` is the most advanced and extensive step, as it implements most of the driving actions of robots. Follow these steps to properly configure your `DriveStep`:

#### 1. Define the ReachCondition

Define whether the DriveSteps finishes by reaching a single target, all targets or never like so:

```cpp
new_step.setReachCondition(DriveStep::ReachCondition::<Condition>);
```

`<Condition>` can either be `NEVER`,`ONE_OF_TARGETS` or `ALL_TARGETS`.

#### 2. (Optional) Define if other robot should be avoided

You can optionally deactivate the avoidance of other robots by using:

```cpp
new_step.setAvoidOtherRobots(false);
```

However, if this should be done is questionable and the default is to avoid other robots.

#### 3. Set RotationControl

Define two which point the robot turns to during the execution of the skill like so:

```cpp
d.setRotationControl(HeadingRotationControl(position));
```

> See [Define Positions](#define-positions) section on how to define the `position` variable.
Don't forget to include the `HeadingRotationControl` Component with

```cpp
#include "local_planner_components/rotation_controls/heading_rotation_control.hpp"
```

#### 4. Add Features

`Features` influence the translational driving of the robot during the `DriveStep`. There are various types of Features:

- Targets: Those define areas the robot wants to be
- AntiTargets: Those define areas the robot wants to drive away from
- CFObstacleFeature: Those define (virtual) objects that the robot should avoid

Every `Feature` has a geometric shape. There are multiple shapes possible:

- Point
- Line
- Circle
- Rectangle
- Arc
- A composition of those listed above

Here is an example for a Target line:

```cpp
d.addFeature(TargetFeature(LineShape(start_positon, end_position)));
```

Like `Steps`, `Features` often have additional configuration parameters. Pay attention to the constructor of the `Features`. An option that every `Feature` has is the weight. The weight is a double between `0` and `1`.

### Define Positions and other Parameters

Constructors of Components require parameters to define the Component and its behavior. Instead of using basic types as `double` or `int` custom classes called `ComponentParam` are used, to provide more flexibility and a powerful API to the skill-developer. These classes represent different types but can be defined in various ways. The following types are implemented with the corresponding class:

- bools: `BoolComponentParam`
- ints: `IntComponentParam`
- doubles: `DoubleComponentParam`
- strings: `StringComponentParam`

Here is an example on how a `DoubleComponentParam` can be defined. All methods also apply for the other types:

```cpp
// using a constant value
DoubleComponentParam p1(3.1417);

// referring to a param from the config store
// the value of the component param is live synced with the config provider
DoubleComponentParam p2(
      config_provider::ConfigProvider::getConfigStore().local_planner_components_config.robot_vel_max_x);

// referring to the TaskData
// here the third double provided in the TaskData is used
DoubleComponentParam p3(TD, 2);

// using a callback that is called every time the value is required (for every control loop cycle)
// this param would always be a random value between 0 and 3 when used in the local planner
// you can use the WorldModel and TaskData provided in the callback to calculated formulas etc.
DoubleComponentParam p4(CALLBACK,
                        [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> double {
                              constexpr double A = 3.0;
                              double random_value = static_cast<double>(rand()) / RAND_MAX;
                              return A * random_value;
                        });

```

Also poses in the WorldModel are presented by `ComponentPosition` and can be defined as follows:

```cpp

RobotIdentifier robot;

// using a frame name
// here the position is defined by the position of the ball frame
ComponentPosition p1("ball");

// using a frame and an offset
// here the position is offest in the "my_frame" frame 1m in x direction and rotated 90° counter clockwise
ComponentPosition p2({"my_frame", 1.0, 0.0, L_PI / 2});
// here the position is offest relative to the robot 1m in x direction and rotated 90° counter clockwise
ComponentPosition p3({robot.getFrame(), 1.0, 0.0, L_PI / 2});

// using a RobotIdentifier
ComponentPosition p4(robot);

// referring to a related robot from the TaskData
// here the position of the second related robot of the TaskData is used
ComponentPosition p5(TD_Pos::ROBOT, 1);

// referring to a point of the TaskData
// here the first position given in the TaskData is used
ComponentPosition p6(TD_Pos::POINT, 0);

// using  callback
// this callback would calculate the mid point between 'frame1' and 'frame2' and return this position also known
// as complex position from the old software if you want to use complex position just use the complexMidPosition
// function from the complex_positions.hpp from the local_planner_components module rather than implementing a
// callback yourself
ComponentPosition p7(
      CALLBACK,
      [](const std::shared_ptr<const transform::WorldModel>& wm, const TaskData& td) -> transform::Position {
      transform::Position p1("frame1");
      transform::Position p2("frame1");
      double alpha = 0.5;

      std::optional<Eigen::Affine2d> t_p1 = p1.getCurrentPosition(wm);
      std::optional<Eigen::Affine2d> t_p2 = p2.getCurrentPosition(wm);
      if (t_p1 && t_p2) {
            Eigen::Vector2d v1 = {t_p1->translation().x(), t_p1->translation().y()};
            Eigen::Vector2d v2 = {t_p2->translation().x(), t_p2->translation().y()};

            Eigen::Translation2d t_mid(v1 + (v2 - v1) * alpha);
            return transform::Position("", t_mid * Eigen::Rotation2Dd(0));
      }
      return transform::Position("");
      });
```

If `ComponentParam` or `ComponentPosition` are defined directly in the constructor of components (without declaring a variable first) the type of the class and the `()` can be replaced with `{}` like so:

```cpp
ComponentPosition start(TD_Pos::ROBOT, 1);
ComponentPosition end({"my_frame", 1.0, 0.0, L_PI / 2});

d1.addFeature(TargetFeature(LineShape(start, end)));

// the lines above can be replaced by:
d1.addFeature(TargetFeature(LineShape({TD_Pos::ROBOT, 1}, {{"my_frame", 1.0, 0.0, L_PI / 2}})));

```
## Examples @subpage skills_examples
