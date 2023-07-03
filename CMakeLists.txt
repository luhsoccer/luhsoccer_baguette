cmake_minimum_required(VERSION 3.16) # Use the newest cmake version that is available on Ubuntu 20.04 LTS

if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.25.0")
    cmake_policy(SET CMP0141 NEW)
endif()

project(
    BOD23
    LANGUAGES C CXX
)

set(DISABLE_TESTS FALSE CACHE BOOL "Disable the building of the tests")
set(DISABLE_BENCHMARKS FALSE CACHE BOOL "Disable the building of the benchmarks")
set(ENABLE_TEST_COVERAGE FALSE CACHE BOOL "Enable the test coverage checker")
set(BUILD_PYTHON_MODULE FALSE CACHE BOOL "Build as a python module")
set(ENABLE_DEPLOY_BUILD FALSE CACHE BOOL "Enables a deploy build. In this build baguette behaves differently")
set(ENABLE_RELEASE_LTO FALSE CACHE BOOL "Enables link time optimization for release builds")

# Sanitizers
set(ENABLE_THREAD_SANITIZER FALSE CACHE BOOL "Enables the thread sanitizer")
set(ENABLE_UB_SANITIZER FALSE CACHE BOOL "Enables the undefined behavior sanitizer")
set(ENABLE_ADDRESS_SANITIZER FALSE CACHE BOOL "Enables the address sanitizer")
set(ENABLE_LEAK_SANITIZER FALSE CACHE BOOL "Enables the leak sanitizer")



# Sets the module for conan
list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR} ${CMAKE_SOURCE_DIR}/cmake_modules)
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR} ${CMAKE_SOURCE_DIR}/cmake_modules)

include(cmake_modules/conan.cmake)

conan_check(VERSION 1.43.0 REQUIRED)

# Enable revisions in the conan config
execute_process(COMMAND ${CONAN_CMD} config set general.revisions_enabled=1
                RESULT_VARIABLE RET_CODE)
if(NOT "${RET_CODE}" STREQUAL "0")
    message(FATAL_ERROR "Error setting revisions for Conan: '${RET_CODE}'")
endif()

set(
    CONAN_LIBS

    "eigen/3.4.0"
    "glm/cci.20220420"
    "spdlog/1.10.0"
    "protobuf/3.21.9"
    "pybind11/2.10.1"
    "asio/1.24.0"
    "tomlplusplus/3.1.0"
    "glfw/3.3.7"
    "imgui/cci.20230105+1.89.2.docking"
    "portable-file-dialogs/0.1.0"
    "tinyobjloader/2.0.0-rc10"
    "implot/0.14"
    "stb/cci.20220909"
    "glad/0.1.36"
    "serial/1.2.1"
    "boost/1.80.0"
)

if(UNIX AND NOT APPLE) # if(LINUX) works only on cmake > 3.25
    message("Overwrite xorg/system dependecy")
    list(
        APPEND
        CONAN_LIBS
        
        "xorg/system@#83ba198a0aade8196d91a2f13e04bb98"
   )
endif()

message(${CONAN_LIBS})

conan_cmake_configure(
    REQUIRES
    ${CONAN_LIBS}
    GENERATORS cmake_find_package
    OPTIONS
    glad:gl_profile=core
    glad:gl_version=4.5
    glad:spec=gl
    glad:no_loader=False
    boost:without_atomic=True
    boost:without_chrono=True
    boost:without_container=True
    boost:without_context=True
    boost:without_contract=True
    boost:without_coroutine=True
    boost:without_date_time=True
    boost:without_exception=True
    boost:without_fiber=True
    boost:without_filesystem=True
    boost:without_graph=True
    boost:without_graph_parallel=True
    boost:without_iostreams=True
    boost:without_json=True
    boost:without_locale=True
    boost:without_log=True
    boost:without_math=True
    boost:without_mpi=True
    boost:without_nowide=True
    boost:without_program_options=True
    boost:without_python=True
    boost:without_random=True
    boost:without_regex=True
    boost:without_serialization=True
    boost:without_stacktrace=False
    boost:without_system=False
    boost:without_test=True
    boost:without_thread=True
    boost:without_timer=True
    boost:without_type_erasure=True
    boost:without_wave=True


)
# Configure conan
conan_cmake_autodetect(settings BUILD_TYPE ${TYPE})
conan_cmake_install(PATH_OR_REFERENCE .
                    BUILD missing
                    UPDATE
                    REMOTE conancenter
                    SETTINGS ${settings})


# This needs to happen after conan setup or else conan will be confused
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

# Create bin directory for executable targets
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

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
    # Use this lines if you want to enable the address sanitizer on linux to find memory leaks
    #add_compile_options(-Wall -Wextra -Wpedantic -fsanitize=address)
    #add_link_options(-fsanitize=address)
    add_link_options(-Wl,--as-needed)
    add_compile_options(-Wall -Wextra -Wpedantic)
    string(REPLACE "-O2" "-O3" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

    if(ENABLE_THREAD_SANITIZER)
        add_compile_options(-fsanitize=thread)
        add_link_options(-fsanitize=thread)
    endif()

    if(ENABLE_UB_SANITIZER)
        add_compile_options(-fsanitize=undefined)
        add_link_options(-fsanitize=undefined)
    endif()

    if(ENABLE_ADDRESS_SANITIZER)
        add_compile_options(-fsanitize=address)
        add_link_options(-fsanitize=address)
    endif()

    if(ENABLE_LEAK_SANITIZER)
        add_compile_options(-fsanitize=leak)
        add_link_options(-fsanitize=leak)
    endif()

endif()
#endif()


# Set some special variables in the build-info.hpp header
set(PROJECT_VERSION_MAJOR 0)
set(PROJECT_VERSION_MINOR 0)
set(PROJECT_VERSION_PATCH 0)
set(PROJECT_VERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH})

# Setups the special header file for versions, etc...
configure_file (
    "${PROJECT_SOURCE_DIR}/include/build-info.hpp.in"
    "${PROJECT_BINARY_DIR}/include/build-info.hpp")

include_directories(${PROJECT_BINARY_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/include)


if(ENABLE_TEST_COVERAGE)
    add_compile_options(--coverage -fprofile-arcs -ftest-coverage)
    add_link_options(--coverage -fprofile-arcs -ftest-coverage)
endif()

# Add the modules to the project
add_subdirectory(extern)
add_compile_definitions(FMT_DEPRECATED_OSTREAM ASIO_NO_DEPRECATED)

if(NOT ENABLE_DEPLOY_BUILD)
    add_compile_definitions(BAGUETTE_LOCAL_MODE="${CMAKE_SOURCE_DIR}")
endif()

add_subdirectory(modules)


if(NOT DISABLE_TESTS)
    enable_testing()    
    add_subdirectory(tests)
endif()

if(NOT DISABLE_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()