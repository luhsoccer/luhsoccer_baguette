# Components {#local_planner_components}
`Components` are the parts a [skill](../skills.md) is build of. There are four types of `Components`:
- Steps @subpage local_planner_steps
- Feature @subpage local_planner_features
- Shapes @subpage local_planner_shapes `coming soon`
- RotationControl @subpage local_planner_rotation_control `coming soon`

They each have an individual purpose, but they are all implemented in the same way. The structure uses the `inheritance` mechanism and the `virtual` and `override` feature of methods. The basic idea is, that each Component type provides a specific method that has defined in and outputs. These methods are `const` which means, that the corresponding object is not changed when the method is called. However, the implementation of these methods can vary from `Component` to `Component`. For example an `PointShape` calculates things different than a `LineShape`, but all `Shapes` are unified by a method that returns the closest point to the robot that is on that shape. The classes that use `Shapes` do not have to know how the specific `Shape` has been implemented, they do not even have to know which specific `Shape` is currently present. Thus, they use a `std::shared_ptr` of `AbstractShape`, which is the base that unifies all `shapes`. The same principles apply for the other Component types.
> Note that the structure of ``Components`` differs heavily from the structure of ``Skills``. `Skills` them selfs are not `Components`, they do not use the inheritance mechanism. While there are different `Skill` objects for different `Skills`, ``Components`` implement different classes for different `Components`

### Parameters
@subpage local_planner_parameters
Parameters that have to be provided at the initialization of ``Components`` (in the constructor) to further define the ``Component`` should always be [ComponentParameters](@subpage parameters.md). 
### Cookies
As State before, ``Components`` only provide `const` methods and therefore they are not able to store information locally. However, some `Components`, like the [WaitStep](@ref WaitStep), need to store information. To accomplish this, `Cookies` have been introduced. `Cookies` are inspired by web browser cookies. A `Component` is able to store data in the [TaskData](../task_data.md) object like a server would do in a browser. All Components can create private space in an [TaskData](../task_data.md) object and store any kind of variables that can be stored in an `std::any`. As only `Components` are allowed to store `Cookies` only they have access to it. For that some `protected` methods have been implemented in the `AbstractComponents` for that. Here are some examples on how to use them inside of methods of `Components`:
```cpp
// store data
// td is the provided TaskData, "start" is a key to identify, and the last argument is the data
this->setCookie(td, "start", time::TimePoint(13.0));

// get time point
// note that the return is optional, because it cannot be guaranteed that any data is present with the provided key
std::optional<time::TimePoint> time = this->getCookie<time::TimePoint>(td, "start");
```