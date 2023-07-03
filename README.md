# Baguette {#mainpage}

`Codename BOD23. Let's win Bordeaux!`

The official luhsoccer software to make goals using many, many actions...

## Important Links:
All of these things are updated when a new change comes to the main branch:
 - [Documentation](https://software.luhbots-hannover.de/)
 - [Test coverage](https://software.luhbots-hannover.de/test_coverage.html)
 - [Python builds](https://gitlab.com/luhbots/luhsoccer_baguette/-/jobs/artifacts/main/browse/wheelhouse?job=build-python-modules)

## Building and IDE support

### Requirements for building

- C++20 Compiler (gcc, clang or msvc)
- Python 3.8+ with pip installed
- CMake 3.16+
- Ninja-Build
- A debugger like GDB or the MSVC debugger
- Clang-tidy
- Xorg-dev (Only on linux, needed for the gui)

### First step
The first step is to clone the repo via git. You need to setup your ssh-keys accordingly. Information about using an ssh-key with gitlab can be found [here](https://docs.gitlab.com/ee/user/ssh.html).

Clone the repo:
```sh
git clone git@gitlab.com:luhbots/luhsoccer_baguette.git --recurse-submodules
```

### Prepare linux environment

On linux the setup of the system is fairly easy: First you need to install the required system packages. Under Ubuntu-22.04 (and 20.04) this can be done via this command: `sudo apt install gcc gdb python3 python3-pip cmake ninja-build clang-tidy xorg-dev`. Next you should run the `./bootstrap.sh` file.

### Prepare Windows Environment

On windows the setup is a little bit more complicated. First you need a compiler. While using MinGw is possible, the recommended option is to use the official Microsoft MSVC Compiler. To install the compiler you need to install [Visual Studio]() first. When installing select the `Desktop development with C++` workload.

Next install the necessary programs to build the software. Download the latest python version from the [official website](https://www.python.org/downloads/). Make sure to select the option to add python to the `PATH` variable on install.

When python installing is done, install the necessary build tools: Open a command prompt and type: `pip install conan==1.59.0 ninja`. After that you also need to install cmake. This can be done from the officall installer or quick via the `winget install -e --id Kitware.CMake` command.

### Import project on VS Code

VS Code development works on linux and windows without any problems. On windows you just need to install the required by yourself.

- Clone the repo via ssh
- Run `./bootstrap` on linux. On windows this step is not necessary. This will create a build folder which can be used by vs code. Also the needed packaged will be installed and/or compiled.
- Open the root folder in VS Code, install the recommended extensions and let the cmake extensions do the rest for you. On windows make sure to select the `amd64` toolchain when asked.
- If you got any problems run the software, check if the cmake launch target is set to `baguette`

### Manual build

- Clone the repo via ssh
- Run `./bootstrap` on linux or `bootstrap.bat` on windows. This will create a build configuration with the debug type.
- Run `ninja -C build` for a build.
- The binaries are now under `build/modules/baguette`

### Building the python modules

The easiest way to use this software is to use it as a python module. To build the software as a python module for every python version (from 3.7 to 3.11) the following steps are necessary:
- Clone the repo: `git clone <ssh or https>`. There should now be a directory called `luhsoccer-baguette`. Don't change into this directory.
- Install the `cibuildwheel` program via pip: `pip install cibuildwheel`
- Build the python modules: `cibuildwheel --platform=linux`
- The build packages are now in the `wheelhouse` folder and can be installed via pip

## Start and Use the Software with the Simulation
After building the software can be started and used with the simulation. For that first start the simulation using the script located in the root directory of the project. On linux use:
```sh
./start_sim.sh
```
On windows use:
```
.\start_sim.bat
```
After `Started er force simulation...` was printed the simulation is started and the software can be started.
Use either the play button in bottom panel of VSCode or start the executable directly. 

After that luhviz should popup. Make sure to set the Vision Source and Robot Connector to `Simulation` and the Simulation Connector to `ErFroce-Simulation-Connector` at the bottom panel of luhviz. Now the field should be displayed in the RenderView.

## Development

[More documentation](modules/README.md) on using libraries and adding software modules can be found in the modules folder.

### Code Style

This project uses the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html). The code style is defined in the [.clang-tidy](.clang-tidy) file and will be applied automatically. Files and directory should follow the `snake_case` style, no uppercase letters allowed. Since this is a modern project all C++ Source files should have to the `*.cpp` extensions and all C++ Header files should have the `*.hpp` extensions.

<!-- @subpage modules_page -->
<!-- @subpage configs
@subpage include_module
@subpage external_depends
@subpage tests -->
