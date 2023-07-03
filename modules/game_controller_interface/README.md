# GameController Tool

Description: This module can be used by luhviz to start the SSL-GameController in a mode in which it **doesnt publish ref commands over multicast/the local network**

Usage: 

```C++
// create a GameController Object 
luhsoccer::game_controller_interface::GameControllerInterface gc("<PATH TO GAME CONTROLLER EXE/BINARY>");
// start the actual process 
gc.startGameController();

<other code here ...>

// send a kill signal to the game controller and wait for it to finish 
gc.stopGameController();
```
