# MarkerService Tool {#marker_service}

This module provides functionallity to create Visual Objects with can be displayed in Luhviz.
These Objects are called Markers. There are to different MarkerTypes which can be displayed.
The normal Markers (3DMarkers) are 3D-Models. There are different Types of available 3DMarkers
e.g. Cube, Robot, Ball or Text. A 3DMarker can be created like in the following example:
(For this is a reference to the MarkerService Object neccessary (Don't create one yourself))

NEW MarkerTypes:

RobotInfoMarker:
```cpp
    // display status information about a robot:
    marker::RobotInfo r{identifier};
    r.setStatus("Connected", marker::Color::Green());
    r2.setStatusTextColor(marker::Color::BLACK());
    r.addParam("Battery Status", 60);
    r.addParam("Receiving Feedback", true);
    r.addParam("sample", "example");
    r.addParam("Kickvoltage", 100.0);
    this->marker_service.displayMarker(r);
```

InfoMarker:
```cpp
    // display whatever value you want to show (pattern: "Name: value") 
    marker::Info i1{"Observer", 1};
    i1.set("ball->goal distance", 3.45);
    this->marker_service.displayMarker(i1);

    marker::Info i2{"Observer", 5};
    i2.set("Goal propability", 1);
    this->marker_service.displayMarker(i2);

    marker::Info i3{"Observer", 2};
    i3.set("running?", true);
    this->marker_service.displayMarker(i3);

```

```cpp
    // display a 3d Robot Model
    marker::Robot robot(position, namespace, id);           // create a 3DMarker of Type "robot"
    robot.setColor(marker::Color::BLUE());                  // set the color (standard color or normalized rgb values)
    robot.setLifetime(3)                                    // disappears after 3 seconds
    marker_service.displayMarker(robot);                    // display the robot
```

The second MarkerType is Marker2D, which provide the function to draw simple 2d shapes.
Example:
```cpp
    // draw the field ground as filled rectangle:
    marker::Rect field_ground(position, "environment", 1);  // create rectangle marker2D
    field_ground.setSize({9, 6});                           // set size in meters
    field_ground.setFilled(true);                           // set this rectangle to fill its inside
    field_ground.setHeight(-0.01);                          // set the height of the marker (the z-direction)
    field_ground.setColor(
        marker::Color::hsv2Rgb(120, 100, 50, 1));          // set the color from hsv values
    // field_ground.setLifetime(0);                         // lifetime = 0 is the default value
    ms.displayMarker(field_ground);                         // display it

    // draw a custom linestrip with n points 
    // (results in n-1 segments)
    marker::LineStrip line(position, "luhviz", 2);          // create a linestrip marker2D
    line.setColors({marker::Color::RED(), 
                    marker::Color::BLACK(), 
                    marker::Color::BLUE(), 
                    marker::Color::YELLOW()});              // assign a color to each segment (default is white)
    line.setPoints({{0, 0}, 
                    {0.5, 0}, 
                    {0.5, 0.3}, 
                    {0.2, 0.6}});                           // set the points of the segments
    line.setPathClosed(true);                               // tell luhviz to connect first and last point
    ms.displayMarker(line);                                 // display it
```

The namespace is used in luhviz UI to disable and enable visual components with one click. So the best practice is
to give markers from the same module or script the same namespace. The given id however must be unique inside 
the same namespace. Otherwise only one object with the duplicated ids will be displayed.
Position and rotation of the markers are controlled by the affine2d position object of the worldmodel component.
All values units are standard units (in real life) like meters, seconds, ...

## Subtopic

