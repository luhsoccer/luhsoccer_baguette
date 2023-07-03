# Transform tool {#transform}

The `transform` tool handles all geometric relations between all robots, the ball and the field. It is inspired by the [tf module](http://wiki.ros.org/tf) of ROS but is optimized for multi threaded, multi robot, dynamic environment applications. In addition to transformations between frames the velocity is handled and stored as well in most cases.

> You should always use the `transform` tool to store, calculated or publish any geometric relations. It is strongly discouraged to store transform or transform vectors by your self as class members or something, as robots tend to move and the transforms can become incorrect quite fast. Please talk to the software leader if you want to do so.

This tool uses the [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) library for vector and matrix operations as well as for storing transform between frames.

## Documentation

1. Terminology @subpage transform_terminology
2. WorldModel @subpage transform_world_model
3. Position @subpage transform_position
