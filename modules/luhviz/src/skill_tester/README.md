# Skill tester & Scenarios
The SkillTester is made for manipulation of the robots and the ball and to send them commands to execute a certain skill. 

## Teleportation - move the ball or the robots
To move the ball or the robots in the simulation one can use the button "telelport ball" (shortcut: "z") or select a robot with the mouse in the render view and use the button "teleport robot" (shortcut: "t").
In the teleportation mode, the robot/ball sticks to the mouse cursor and is placed with another click of the mouse at the new location.

## Skill testing
If an ally robot is selected, there can be choosen one of the available skills in the dropdown, e.g. goToPoint or getBall. The list of available skills is loaded indirectly by calling the DataProxy which loaded the available skills from the skillbook at program startup. For every skill there exists certain taskdata which are predefined parameters needed to execute the skills. That is for example the position for goToPoint or the ally robot id for passBallToRobot. Which parameters are needed for the current selected skill is polled by the DataProxy und for this the needed input fields (depending on the parameter variable type) are dynamically rendered. That results to not very good readable code because all params are dynamic sized lists. As there are also skills which just accept a list with a non predefined size, the user can for this inputs click on the plus button to create another input field for that type. 
The inputs made are mostly checked before calling the skill. If some parameter is missing the skill cant be executed. The shortcut for executing the skill is "x" for execute.

## Scenario testing
Scenarios can also be started via a dropdown and a button at the bottom. One can choose a scenario from the list in the dropdown and than start it. There is also the possibility to set the number of repetitions so the scenario is looped x times. 