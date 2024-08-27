# luhsoccer_baguette Development Guide

This guide gives an overview how to setup the baguette repo on your local machine.

## Setup the development environment (Linux)

The following steps are required to setup the baguette repo:

1. Install the required dependencies

    The following step depends on your Linux distribution. Choose the correct section for your distribution.

     The following distributions are tested and should work out of the box:

    - Ubuntu 22.04
    - Fedora 39

    If your distribution is not listed, it should be possible to install the dependencies manually. You can ask a member of the team for help. A different version of Ubuntu or Fedora should also work, but is not tested.

    - Ubuntu 22.04
        - First you need to update your system:

            `sudo apt -y update && sudo apt -y upgrade`

        - Then you can install the required dependencies:

            `sudo apt -y install git cmake curl zip unzip tar g++ pkg-config libglu1-mesa-dev libxinerama-dev libxcursor-dev libxrandr-dev libxi-dev ninja-build python3-dev python3-pip python3-setuptools-scm perl`

    - Fedora 39
        - First you need to update your system:

            `sudo dnf upgrade --refresh`

        - Then you can install the required dependencies:

            `sudo dnf install -y cmake g++ xorg-x11-server-devel libX11-devel libXrandr-devel libXinerama-devel libXcursor-devel libXi-devel mesa-libGL-devel ninja-build python3-devel perl`

    - Windows 11
        - First make sure your system is up-to-date.

        - Download and install [Visual Studio 2022 Community Edition](https://visualstudio.microsoft.com/de/)
            - When installing Visual Studio make sure you select *Desktop Development with C++*.

        - Install [Python](https://www.python.org/downloads/) and check for *Add python.exe to PATH* at the end of the process.

        - Open a command-line and install *ninja* using the command:

            `pip install ninja`

        - Install [git](https://git-scm.com/download/win)

        - Install [CMake](https://cmake.org/download/)

2. Generate an SSH key

    In order to clone the repo, you need to generate an SSH key and add it to your GitLab account. The following guides can help you with this:

    - [Generate a key](https://docs.gitlab.com/ee/user/ssh.html#generate-an-ssh-key-pair)
    - [Add the key to your GitLab account](https://docs.gitlab.com/ee/user/ssh.html#add-an-ssh-key-to-your-gitlab-account)

3. Clone the repo with submodules

    `git clone git@gitlab.com:luhbots/luhsoccer_baguette.git --recurse-submodules`

    If git asks you to verify the host, type `yes` and press enter.

4. Prepare the first build

    Change into the repo directory:
    `cd luhsoccer_baguette`

    Configure the project to install all required development dependencies:

    `cmake --preset Debug/gcc`

5. Build the project

    `cmake --build build/Debug/gcc`

6. Run the project

    `./build/Debug/gcc/modules/baguette/baguette_static_Debug`

    When everything worked, you can continue with the [VS Code Setup](#vs-code-setup).

## VS Code Setup

We recommend using VS Code as IDE for this project. The following steps are required to setup VS Code:

1. Install VS Code

    Download and install the latest version of VS Code from the [official website](https://code.visualstudio.com/).

2. Open the project in VS Code

    Open VS Code and click on `File -> Open Folder...`. Select the `luhsoccer_baguette` folder and click on `Open`.

3. Install the recommended extensions

    VS Code will show a notification that recommends installing the recommended extensions. Click on `Instal` to install them. If you missed the notification, you can also install them manually:
    - C/C++
    - C/C++ Extension Pack
    - CMake Tools
    - clangd
    - Build Output Colorizer

4. Setup CMake and clangd

    In the toolbar at the bottom of the window, click on `No Configure Presets Selected` and choose a profile for the project. We recommend using `Debug/gcc` for development and `ReleaseWithDebInfo/gcc` for testing.

    When opening a C++ file for the first time, clangd will ask you to download the language server. This is needed for code completion and other features. Click on `Download` to install it.

5. Run baguette

    In the toolbar at the bottom of the window, click on the arrow to build and launch the project.

6. Start developing

    You can now start developing. If you have any questions, feel free to ask a member of the team.
    Information about the module structure can be found in the [Modules Guide](modules/README.md).

## Development with Sanitizers

Since C++ is a complex language, it is easy to make mistakes. To help you find these mistakes, we recommend using sanitizers. Sanitizers are tools that help you find bugs in your code. The following sanitizers are available:

- AddressSanitizer (ASan)
- LeakSanitizer (LSan)
- UndefinedBehaviorSanitizer (UBSan)
- ThreadSanitizer (TSan)

Only **one** sanitizer should be used at a time. To enable a sanitizer, you need to set the `ENABLE_<NAME>_SANITIZER` variable in the CMake configuration. In VSCode this can be achieved by using the 'CMake: Edit CMake Cache (UI)' command. In the UI, search for the sanitizer you want to enable and set the variable to `ON`. After that, you need to reconfigure the project and rebuild it.

The runtime library may need to be installed on your system. On Fedora you can
install the runtime libraries with the following command:

```bash
sudo dnf install -y libasan libubsan libtsan liblsan
```

To use the sanitizers when building the python bindings the follow commands can be used:

- `pip install . -vvv --config-settings=cmake.define.ENABLE_ADDRESS_SANITIZER=TRUE` to enable the AddressSanitizer
- `pip install . -vvv --config-settings=cmake.define.ENABLE_UB_SANITIZER=TRUE` to enable the UndefinedBehaviorSanitizer
- `pip install . -vvv --config-settings=cmake.define.ENABLE_THREAD_SANITIZER=TRUE` to enable the ThreadSanitizer
- `pip install . -vvv --config-settings=cmake.define.ENABLE_LEAK_SANITIZER=TRUE` to enable the LeakSanitizer

When starting python with the sanitizers enabled, the library must be loaded before any other code executes. To archive this, the `LD_PRELOAD` environment variable can be used. The following commands can be used to start python with the sanitizers enabled:

- First you need to find name of the sanitizer library. The following commands can be used to find the library:

  - `find /usr/lib64 -name 'libasan.so*'` to find the AddressSanitizer library
  - `find /usr/lib64 -name 'libubsan.so*'` to find the UndefinedBehaviorSanitizer library
  - `find /usr/lib64 -name 'libtsan.so*'` to find the ThreadSanitizer library
  - `find /usr/lib64 -name 'liblsan.so*'` to find the LeakSanitizer library

- Then you can start python with the sanitizer enabled (for example if the library ends with `.6`):
      - `LD_PRELOAD=/usr/lib64/libasan.so.6 python` to start python with the AddressSanitizer
      - `LD_PRELOAD=/usr/lib64/libubsan.so.6 python` to start python with the UndefinedBehaviorSanitizer
      - `LD_PRELOAD=/usr/lib64/libtsan.so.6 python` to start python with the ThreadSanitizer
      - `LD_PRELOAD=/usr/lib64/liblsan.so.6 python` to start python with the LeakSanitizer

## Code Style

This project uses the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html). The code style is defined in the [.clang-tidy](.clang-tidy) file and will be applied automatically. Files and directory should follow the `snake_case` style, no uppercase letters allowed. Since this is a modern project all C++ Source files should have to the `*.cpp` extensions and all C++ Header files should have the `*.hpp` extensions.

## Development with Python

The project has bindings for Python. They are automatically compiled when building the project. To test the bindings, you can install the current source code as a Python package with `pip`:

In the project root directory, run: `pip install . -vvv` to install the package. You can now import the package in Python with `import baguette_py`. If you make changes to the Python bindings, you need to reinstall the package with `pip install . -vvv`.

If you want python to automatically reload the package when you make changes, you can use the `pip install --no-build-isolation . -evvv` command. This will install the package in editable mode. You can now import the package in Python with `import baguette_py`. If you make changes to the Python bindings, you don't need to reinstall the package. Just restart the Python interpreter and the changes will be applied.
