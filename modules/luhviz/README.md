# Luhviz Module {#luhviz}
Luhviz contains the visualization of our Software/Strategy. Therefore it has a really complex structure with a lot code compared to many other modules.
At the base luhviz is using OpenGL to to render graphical content to the screen. For this, the utility library GLFW is used.
On top of GLFW Luhviz uses the ImGui Docking library as the GUI-Framework which comes with predefined Elements like Buttons, Dropdowns etc.

## Luhviz Main Class
The main class is further explained in [Luhviz Main Class](./src/README.md).

## Data Proxy - organize dependencies to other software modules
Luhviz as a module is a little different to all other modules, because at the end every user input to the whole software will start at some point in the UI in luhviz. Therefore luhviz has dependencies to almost all other modules. To organize function calls of other modules and keep track of data belonging to other modules we introduced the so called DataProxy. It is used in many luhviz windows to communicate to the local planner / skill system, the config provider, the game data provider and other parts of the software. As an example of the SkillTester, the DataProxy keeps track of the current selected skill, the input task data and the task data needed for that skill. At the end, the SkillTester itself only communicates with the DataProxy and therefor only indirectly depends on the localplanner / skillsystem.


## Main Window - the base window
The general layout consists of the [Main Window](./src/main_window/README.md), which is the parent window.
All other modules in the source folder are dockable subwindows which the user can position as desired.
The docking mechanism itself is part of the ImGui Framework.
At the bottom of the this window there are dropdowns for choosing the source Configurations for the vision source, simulation connector, robot connector, vision upstream and game log source.

## Render View - the heart of luhviz
This is the main component in for displaying a extended abstraction of the real game field with robots and the ball. 
Additionally there are many possebilities to display additional content which are named the markers from the [MarkerService](../marker_service/README.md)
which is a different module. Internally the RenderView uses OpenGL to render 3d models and 2d shapes in this window.
This is explained in detail in the [New Rendering](./src/new_rendering/README.md).
The 3D content is rendered into an image with anti aliasing applied for sharp edges. Finally the image is displayed in an ImGui component.
On top there are some control buttons for switching between the 2D top view and 3D perspective. 
The render view itself is explained in [RenderView](./src/render_view/README.md).
In the [Common](./src/common/README.md) folder there is the controls script which handles how the user can navigate inside the RenderView. It handles mouse drag to rotate the camera and much more. The Physics script is used to perform raycasts into the 3D rendering scene and therefore is used to select robots with the mouse cursor.

## Inspector - choose what information to display
With this tool it is easy to show and hide markers while luhviz is running. Every marker is categorized with a namespace and an id which is displayed in this tool as a checkbox expandable tree. More information can be found in [Inspector](./src/inspector/README.md)

## Config Window - live parameter configuration
The [Config](./src/luhconfig/README.md) windows main purpuse is to easily change configuration parameters for the whole software and strategy while running. The parameters are organized in different tabs per module and inside a tab they are grouped by the given config group name.

## Robot Display - show robot hardware status
The [Robot Display](./src/robert_display/README.md) is made for displaying hardware information of the robots on the field. Especially there is a status bar showing the connection status and various other data like battery voltage and current velocity.

## Info Display - show important software/strategy information
The [Info Display](./src/info_display/README.md) is very usefull to display information from modules at the current game status. There is the possibility to group them by given namespace and to colorize them.

## Skill Tester & Scenarios - test robot behaviour
With the [Skill Tester](./src/skill_tester/README.md) the user can manipulate contents in the RenderView such as teleporting the robots and the ball and assigning [skills](../skills/README.md) to the ally robots. The additionally needed information for the selected skill can be filled in and than the skill can be executed. There is also the posibility to run [Scenarios](../scenario/README.md) which are predefined situations which than will be executed. 

## Console - logging, warnings, errors
The [Console](./src/debugger/README.md) displays all print outs from the [Logger](../logger/README.md) module. There can be set various filters to display which logging level and to filter for keywords or even filter for the modules the messages are coming from.

## Game Info - show current Game Controller status
The [Game Info](./src/game_info/README.md) displays some information gathered from the connected Game Controller. It is very usefull to see if the software is connected successfully to the game controller. It displays the current game command, team names and logos, the score and various other information.

## Game Controller Window - Game controller utility tool
In the [Game Controller](./src/game_controller_window/README.md) window the user can start the official ssl game controller software inside luhviz.

## Plotter - visualize data with plots
The [Plotter](./src/plotter/README.md) is a special module which can be used in combination with plot markers to display real time plots in a graph or multiple graphs. As default the plotter shows a graph for the velocity of the ball and the robots.

## Timeline - record games and relplay them
This component is still under development. More infos are in [Timeline](./src/timeline/README.md)

## Skill Wizard - simple editor to create/change skills
This component is still under development. More infos are in [SkillWizard](./src/skill_wizard/README.md)