# Robot Display
The robot display receives a special type of marker from the [MarkerService](../../../marker_service/README.md) which is called the **RobotInfoMarker**. All this luhviz module does is to render that information in a pretty way to the screen. In detail, there is drawn a rounded corners box for every robot and inside of that box the information for that robots are displayed respectively.

The information include the following data: 
- Connection status - Connected, if the robot receives commands and sends feedback
- Battery voltage - the volage level of the battery 
- Cap voltage - the voltage of the capacitors of the kicker
- Has ball - from the light barrier sensor at the dribbler
- Velocity - from the feedback of the robots
- Vision coordinates - from the vision system