# Luhviz Main Class

## Setup
The setup happens in the init method. It sets up the glfw shit and create the glfwWindow. Also all luhviz modules are created and initialized. While all these ressource are loaded (images, shaders, ...) a splashscreen with the luhbots logo is displayed (about 1 sec).  
For GLFW there is configured an error callback, an opengl debug context for debugging the shaders and the OpenGL version 4.5 is set.
Also we set the number of samples to take to 4 to enable MSAA antialiasing for a sharper rendered outcome. The GLFW window is set to maximize and after that it is finally created. The window size limits are also set to prevent problems. According to the config param setting vsync is activated or deactivated with setting the swap intervall


## RenderLoop
The main rendering loop is created in the runLuhviz method. 
Here the ImGui context is created and the window layout file is loaded. Either there is an existing one or the default one will be loaded from the [CMAKE_RC](../../../extern/cmake_rc/README.md) filesystem. Our custom ImGui window look and style is applied and the fonts are loaded. After that the loop starts. In the loop all the modules render() methods are called and the DataProxys update method is called.