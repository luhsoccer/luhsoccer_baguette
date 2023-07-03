# Python Bindings Module {#python_bindings}

This guide you help you to create simple python bindings for your own module or tool. The way to add bindings differs for these two options.


## Module
To add python bindings for a module of the software, you need to register the module to the bindings. The needs to happen in the `module_bindings.cpp` file. Add the following line of the code add your bindings:

```cpp
    loadModuleBindings(baguette_module, wrapper, *baguette.module_name, "MODULE_NAME");
```

After that you need to create a new file which contains the bindings. Per conventions this file should be included in the `bindings` folder. 

## Tool
To add python bindings for a tool (for example helper classes that are completely static), you don't need to register a module. You can simple add the required functions and classes in the `tool_bindings.cpp`