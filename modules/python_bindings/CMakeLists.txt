set(MODULE_NAME
    _baguette_py
)

find_package(
    Python 3.8
    REQUIRED COMPONENTS Interpreter Development.Module
    OPTIONAL_COMPONENTS Development.SABIModule
)


find_package(MPMCQueue REQUIRED)
find_package(nanobind CONFIG REQUIRED)
find_package(TBB CONFIG REQUIRED)
find_package(magic_enum CONFIG REQUIRED)

nanobind_add_module(
    ${MODULE_NAME}
    STABLE_ABI
    NOSTRIP
    src/python_module.cpp
    src/tool_bindings.cpp
    src/module_bindings.cpp
    src/bindings/common.cpp
    src/bindings/event_system.cpp
    src/bindings/game_data_provider.cpp
    src/bindings/config_provider.cpp
    src/bindings/ssl_interface.cpp
    src/bindings/transform.cpp
    src/bindings/logger.cpp
    src/bindings/marker_service.cpp
    src/bindings/robot_interface.cpp
    src/bindings/simulation_interface.cpp
    src/bindings/software_manager.cpp
    src/bindings/skills.cpp
    src/bindings/local_planner.cpp
    src/bindings/task_manager.cpp
    src/bindings/util.cpp
)


target_link_libraries(${MODULE_NAME} PRIVATE magic_enum::magic_enum)

target_link_libraries(${MODULE_NAME} PUBLIC
    baguette_static
    TBB::tbb
    TBB::tbbmalloc
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_compile_definitions(${MODULE_NAME}
    PUBLIC
)

if(MSVC)
    install(FILES $<TARGET_PDB_FILE:${MODULE_NAME}> DESTINATION baguette_py OPTIONAL)
endif()

install(CODE "set(STUBGEN_SOURCE \"${CMAKE_SOURCE_DIR}/modules/python_bindings/stubgen.py\")")
install(TARGETS ${MODULE_NAME} DESTINATION baguette_py)

if(NOT DISABLE_STUBGEN)
    install(SCRIPT GenerateStubs.cmake)
endif()

# Can't enable pch here since we're building a python module
#target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)