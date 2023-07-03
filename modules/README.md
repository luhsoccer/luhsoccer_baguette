# Modules {#modules_page}

Heres is the place were all the software modules go. Each module has it own subfolder but can have dependencies to all other modules.


* @subpage baguette_module
* @subpage game_data_provider
* @subpage luhviz
* @subpage role_manager
* @subpage python_bindings
* @subpage skills
* @subpage local_planner_components_module
* @subpage local_planner
* @subpage robot_interface
* @subpage simulation_interface
* @subpage ssl_interface
* @subpage scenario
* @subpage experiment_logger
* @subpage marker_service
* @subpage time
* @subpage logger
* @subpage transform
* @subpage config_provider
* @subpage include_module
* @subpage template_module
## Structure
### File structure

```txt
modules/
├─ module_1/
│  ├─ include/
│  │  ├─ module_1/
│  │  │  ├─ public_header_2.hpp
│  │  │  ├─ public_header_1.hpp
│  ├─ src/
│  │  ├─ private-header.hpp
│  │  ├─ source_file_1.cpp
│  │  ├─ source_file_2.cpp
│  ├─ CMakeLists.txt
│  ├─ README.md
├─ module-n/
│  ├─ include/
│  │  ├─ module_n/
│  │  │  ├─ public_header_n.hpp
│  ├─ src/
│  │  ├─ private_header_n.hpp
│  │  ├─ private_source_file_n.cpp
│  ├─ CMakeLists.txt
│  ├─ README.md
├─ CMakeLists.txt
├─ README.md
```

As you can see each module is into a custom directory. Each of this module-directory should contains the following items

- A `include` directory. Here is the place for the public header of the modules. (The headers that can/should be used by other modules.) The header itself should go into a directory with the module for better information later to which module a header belongs.
- A `src` directory. All source file and private headers should be here. No rules here for naming since the contents of the folder are not visible by other modules.
- The `README.md` file. Each module should have a short description about the functionality and scope of module, so new developers can easy get an overview over the project.
- `CMakeLists.txt`. This file tells `CMake` how to build the modules. It must be created with the module and only needs to be changed when new files are needed to the module. This is (sadly) necessary since the `glob` operator of `CMake` can lead to problems.

### Add a file to a module

In case you create a new file (source or header) you need to add it to the `CMakeLists.txt` of the module. This is necessary to tell the compiler which source-files should be compiled and the IDE which files belong the project (source and header). To add a file to `CMake` just add the relative file path from the module root to the `add_library(...)` or `add_executable(...)` function. For the module-name you can always to the `${MODULE_NAME}` variable, it's defined in all modules of this software. For the example file structure in the top of `module-1` it would look like this:

```cmake
add_library(${MODULE_NAME}
  src/private-header.hpp
  src/source_file_1.cpp
  src/source_file_2.cpp
  include/${MODULE_NAME}/public_header_1.hpp
  include/${MODULE_NAME}/public_header_2.hpp
)
```

### Add a dependencies to a module

There is a difference in adding a dependency to an external library or a simple
dependency to another module:

#### Dependency to another module

If you want to add an dependencies to a module of this project just add the name of the module to the `target_link_libraries(...)` function in the `CMakeLists.txt`. If the function does not exists create it using the following template:

```cmake
target_link_libraries(${MODULE_NAME} PUBLIC
    module1
    module2
)
```

where `module1` and `module2` are the needed dependencies of the module. `${MODULE_NAME}` doesn't need to be changed since it is defined for every `CMakeLists.txt` of the modules.

#### Dependency to an external library

To have an dependency to an external library some more steps are required. First you must ensure that the library is added in the [root CMakeLists.txt](../CMakeLists.txt) in the `REQUIRES` parameter list. If it is not the case and you need to the find the library in the [conan package registry](https://conan.io/center/). Simply add the library with the version to the argument list. For example:

```cmake
conan_cmake_configure(REQUIRES
    eigen/3.4.0
    GENERATORS cmake_find_package
)
```

if you want to add eigen with version 3.4.0.

When the library has now been added or was already present adding the dependency to the module is fairly simple: Just like in the [section before](#dependency-to-another-module) you need to add to the `target_link_libraries(...)` function. The only difference is that you first need to use the `find_package(...)` function. For example, if you added `myLib/2.3.0` to the conan config the resulting `CMake` function will look like this:

```cmake

find_package(myLib REQUIRED)

target_link_libraries(${MODULE_NAME} PUBLIC
  myLib::myLib
)
```

The exact name in the `find_package(...)` function may differ from the conan library name. This has comparability reasons. For the name to use follow the documentation of the library you want to add.

### Add a module

To add new module to the software create the subdirectory with the module name and insert a [file-structure like above](#file-structure). The `README.md` should contain a brief description about the function of the module. The `src` and `include` contain the source and header file of the module. The `CMakeLists.txt` is the most important part when it comes to adding a new module. But only a few steps are required in order to make it work:

1. Create the `${MODULE_NAME}` variable. In order for the next commands to work you need to create a variable with the name of the module. Add the following code as the first line in the `CMakeLists.txt`:

     ```cmake
    set(MODULE_NAME
        YourNewModuleName
    )
    ```

2. Create the target. Here you tell `CMake` that your module should be static linked into the main executable. As described [before](#add-a-file-to-a-module) this is also the part where you add the source and header files:

    ```cmake
    add_library(${MODULE_NAME} STATIC
        src/test.cpp
        include/${MODULE_NAME}/test.hpp
    )
    ```

3. Set the include directories. If you want that other modules can use your code you need to set the public include path of your module. You maybe also want to set a private include path so that you can define private headers that are only visible in your module.

    ```cmake
    target_include_directories(${MODULE_NAME}
        PRIVATE src
        PUBLIC include
    )
    ```

4. Finished! For example the `CMakeLists.txt` for `module-1` of the example file structure would look like this:

    ```cmake
    set(MODULE_NAME
        module_1
    )

    add_library(${MODULE_NAME}
      src/private_header.hpp
      src/source_file_1.cpp
      src/source_file_2.cpp
      include/${MODULE_NAME}/public_header_1.hpp
      include/${MODULE_NAME}/public_header_2.hpp
    )

    target_include_directories(${MODULE_NAME}
        PRIVATE src
        PUBLIC include
    )
    ```

    But in order for it to be found by the root project it's still necessary to add your module directory to the [`CMakeLists.txt` in the `modules` folder](CMakeLists.txt). Just add a new line with `add_subdirectory(yourModuleName)` at the end of the file. Now your module is available in the project an other can use it.
