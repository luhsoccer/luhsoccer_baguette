# Position class {#transform_position}
The ``position`` class represents a position in a world model. It is defined by a frame in which the data is provided and an x,y and rotation coordinate.

> While [Frames](terminology.md#frames) are meant to move over time, `Position` objects are meant to be static relative to the provided frame. If you want to represent a moving position create a frame in the [WorldModel](world_model.md) and create a `Position` object with all coordinates zero (default).


The `Position` class provides methods to get the current [Transform](terminology.md#transform) of the `Position`-Object in a given [WorldModel](world_model.md). The function has to be called in every control loop, as frames tend to move and the results from the last control loop are most probably invalid.

Here is an example of how to use this method to get the transform to the position in the global frame at the latest available point in time:
```cpp
// create new position object
transform::Position position("frame_name", 1.0, 0.0, 3.141);

// get the current latest (time::TimePoint(0)) transform in the global frame (empty string)
std::optional<Eigen::Affine2d> transform = position.getCurrentPosition(wm, "", time::TimePoint(0));
```
> See [Transform](terminology.md#transform) on how to handle `Eigen::Affine2d` objects

It is also possible to get the velocity:
```cpp
std::optional<Eigen::Vector3d> velocity = position.getVelocity(wm, "global_frame", "global_frame", time::TimePoint(0));
```
Here the velocity relative the to global frame (1st "global_frame") of the position is requested. The velocity is returned in the coordinate system of the global frame (2nd "global_frame").
> See [Velocity](terminology.md#velocity) on how to handle `Eigen::Velocity3d` objects.

