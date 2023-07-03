set(MODULE_NAME
    scenario
)

add_library(${MODULE_NAME} STATIC
    src/scenario.cpp
    src/scenario_book.cpp
    src/scenario_executor.cpp
    include/${MODULE_NAME}/scenario.hpp
    include/${MODULE_NAME}/scenario_book.hpp
    include/${MODULE_NAME}/scenario_executor.hpp
    )

#find_package(MyLib REQUIRED)

target_link_libraries(${MODULE_NAME}
    PRIVATE
    robot_interface
    PUBLIC
    local_planner
    skills
    transform
    time
    simulation_interface
    experiment_logger
    
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)