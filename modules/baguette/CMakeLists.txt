set(EXECUTABLE_NAME
    baguette
)

set(MODULE_NAME
    baguette_static
)


add_library(${MODULE_NAME} STATIC
    src/baguette.cpp
    include/baguette.hpp
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)


target_link_libraries(${MODULE_NAME} PUBLIC
    ssl_interface
    simulation_interface
    game_controller_interface
    config_provider
    logger
    transform
    skills
    robot_interface
    game_data_provider
    role_manager
    robot_interface
    luhviz
    luhviz::rc
    scenario
    task_manager
    exception_handler

    PRIVATE
)


add_executable(${EXECUTABLE_NAME}
    src/main.cpp
)

target_link_libraries(${EXECUTABLE_NAME} PUBLIC
    ${MODULE_NAME}
)

# Some more properties which copies the target file into the bin folder.
set_target_properties(${EXECUTABLE_NAME} PROPERTIES OUTPUT_NAME ${MODULE_NAME}_$<CONFIG>)


# Can't reuse precompiled headers since this is an executable
# target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)