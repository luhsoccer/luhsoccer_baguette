# GameDataProvider Module {#game_data_provider}

This modules act as a copy-only template for new modules. If you want to add a new modules just copy this and rename all occurrences from `__template`  to your new module name.
blablabla


# Observer

The Observer is used to analyze the current game state. 

It can be used in two ways: 

**Static Observer:** 
: The Static Observer can be used to calculate Data for a given Robot handle in the thrat of the caller. This can be usefull if you want Observer-Data for a specific World Model. 
: The functions of the static Observer can be called by anyone who has access to a const WorldModel and takes RobotHandles or a WorldModel. 

: It can be found under \
<u>luhsoccer::observer::calculation</u> \
in the file \
<u>observer/static_observer.hpp</u> 

**Continuous Observer:**
: The Continuous Observer can be used to quickly get infos about the Game based on the Global/Main World Model. 

: It can be obtained by the GDP and updates its data every time new Data is available.  

: It can be found under \
<u>luhsoccer::observer::Observer</u> \
in the file \
<u>observer/continuous_observer.hpp</u>