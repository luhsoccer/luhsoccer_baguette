# Common

## Controls - navigate the camera in 3D/2D space
This script uses matrix math to calculate the current cameras position and rotation and calculates its view frustrum. Implemented navigations are:
- Mouse left click drag - rotate around pivot
- Shift + Mouse left drag - translate the pivot point
- Mouse scrollwheel - zoom in and out

## Physics - raycasts and collisions
This script is used for casting rays from the mouse position in the 3D/2D simulation space to then calculate a collision with that ray and some of the robots hitboxes. This mechanism allows to select robots in the RenderView.