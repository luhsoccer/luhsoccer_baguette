cmake_minimum_required(VERSION 3.22) # Use the newest cmake version that is available on Ubuntu 22.04 LTS

# Add directory of GitVersion to the cmake module path.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.25.0")
    cmake_policy(SET CMP0141 NEW)
endif()

include(CheckSubmodule)

check_submodule_git(extern/vcpkg f7423ee180c4b7f40d43402c2feb3859161ef625)

project(
    BOD23
    LANGUAGES C CXX
)
set(DISABLE_STUBGEN FALSE CACHE BOOL "Disable the generation of python stubs")

set(DISABLE_TESTS FALSE CACHE BOOL "Disable the building of the tests")
set(DISABLE_BENCHMARKS FALSE CACHE BOOL "Disable the building of the benchmarks")
set(ENABLE_TEST_COVERAGE FALSE CACHE BOOL "Enable the test coverage checker")
set(ENABLE_DEPLOY_BUILD FALSE CACHE BOOL "Enables a deploy build. In this build baguette behaves differently")
set(ENABLE_RELEASE_LTO FALSE CACHE BOOL "Enables link time optimization for release builds")

# Sanitizers
set(ENABLE_THREAD_SANITIZER FALSE CACHE BOOL "Enables the thread sanitizer")
set(ENABLE_UB_SANITIZER FALSE CACHE BOOL "Enables the undefined behavior sanitizer")
set(ENABLE_ADDRESS_SANITIZER FALSE CACHE BOOL "Enables the address sanitizer")
set(ENABLE_LEAK_SANITIZER FALSE CACHE BOOL "Enables the leak sanitizer")

set(CMAKE_CXX_STANDARD 20) # Use c++ 20 for everything
set(CMAKE_CXX_STANDARD_REQUIRED ON) # Don't allow anything below that
set(CMAKE_CXX_EXTENSIONS OFF) # No compiler-specific extensions to the language. Everything should be compiler independent
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Export compile command to support IDEs like vscode or clion
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # TODO change this to only build PIC when the python module is build. Use static lib otherwise

if(ENABLE_RELEASE_LTO)
    # Enable LTO for release builds. In debug builds it takes too much time
    message("Building with LTO")
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON) # Enable LTO
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO ON) # Enable LTO
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_MINSIZEREL ON) # Enable LTO
endif()

set(MSVC_DEBUG_INFORMATION_FORMAT Embedded)

# Add the modules to the project
add_subdirectory(extern)

# Enables all warnings. In a CI enviroment we want also all warnings as error
#if(DEFINED ENV{CI})
#    if(MSVC)
#        add_compile_options(/W4 /WX)
#    else()
#        add_compile_options(-Wall -Wextra -Wpedantic -Werror)
#    endif()
#else()
if(MSVC)
    add_compile_options(/W4 /wd4458 /we4099 /diagnostics:caret)
    add_compile_definitions(_ENABLE_EXTENDED_ALIGNED_STORAGE)
    string(REPLACE "/Ob1" "/Ob2" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
    # use this lines to generate a pdb when compile in release mode
    #add_compile_options(/W4 /wd4458 /Z7 /DEBUG:FULL)
    #add_link_options(/DEBUG:FULL)

    if(ENABLE_THREAD_SANITIZER OR ENABLE_UB_SANITIZER OR ENABLE_ADDRESS_SANITIZER OR ENABLE_LEAK_SANITIZER)
        message(FATAL_ERROR "Sanitizers are not supported on windows")
    endif()

else()

    # Enable colored output for ninja
    if(${CMAKE_GENERATOR} STREQUAL "Ninja")
        add_compile_options(-fdiagnostics-color)
    endif()


    # Use this lines if you want to enable the address sanitizer on linux to find memory leaks
    #add_compile_options(-Wall -Wextra -Wpedantic -fsanitize=address)
    #add_link_options(-fsanitize=address)
    add_compile_options(-Wall -Wextra -Wpedantic) #-ftime-trace
    string(REPLACE "-O2" "-O3" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

    if(ENABLE_THREAD_SANITIZER)
        message("Enable thread sanitizer")
        add_compile_options(-fsanitize=thread)
        add_link_options(-fsanitize=thread)
    endif()

    if(ENABLE_UB_SANITIZER)
        message("Enable undefined behavior sanitizer")
        add_compile_options(-fsanitize=undefined)
        add_link_options(-fsanitize=undefined)
    endif()

    if(ENABLE_ADDRESS_SANITIZER)
        message("Enable address sanitizer")
        add_compile_options(-fsanitize=address)
        add_link_options(-fsanitize=address)
    endif()

    if(ENABLE_LEAK_SANITIZER)
        message("Enable leak sanitizer")
        add_compile_options(-fsanitize=leak)
        add_link_options(-fsanitize=leak)
    endif()

endif()
#endif()

include(GitVersion)

if(ENABLE_TEST_COVERAGE)
    add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
    add_link_options(--coverage -fprofile-arcs -ftest-coverage)
endif()

add_compile_definitions(ASIO_NO_DEPRECATED TOML_EXCEPTIONS=0 TOML_ENABLE_FORMATTERS=1 TOML_HEADER_ONLY=0)

if(NOT ENABLE_DEPLOY_BUILD)
    add_compile_definitions(BAGUETTE_LOCAL_MODE="${CMAKE_SOURCE_DIR}")
endif()

add_subdirectory(modules)

if(NOT DISABLE_TESTS)
    add_subdirectory(tests)
endif()

if(NOT DISABLE_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()