set(MODULE_NAME
    task_manager
)

add_library(${MODULE_NAME} STATIC
    src/task_manager.cpp
    include/${MODULE_NAME}/task_manager.hpp
)

#find_package(MyLib REQUIRED)

target_link_libraries(${MODULE_NAME}
    PRIVATE
    role_manager
    game_data_provider
    skills
    PUBLIC
    local_planner
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)