# Main Window

## General
In detail, the main window is divided in 4 different vertically divided areas. The first is the **Top Bar Menu**. Its purpose is to show the shortcuts of luhviz and let the user toggle between fullscreen render view and reopen closed windows. The second area is the **Toolbar** directly below the menu. It is filled with different buttons which can be usefull for quick access if the user does not want to use the corresponding shortcut. At the bottom there is the **Bottom Bar**. It displays the current fps and the current used software version. Also one can choose here between different sources to get the simulation or real game data. In the middle is the **Dockspace** for all other windows. Its size is variable and consumes all remaining space which is not occupied by the menu, the toolbar or the bottom bar.

## Window layout handler
The window layout handler keeps track of all opened and docked windows inside this main window. In the top application bar one can reset the whole layout to the default or reopen certain windows which were closed. It also handles toggling RenderViews fullscreen display. 