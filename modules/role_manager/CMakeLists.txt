set(MODULE_NAME
    role_manager
)

add_library(${MODULE_NAME} STATIC
    src/role_manager.cpp
    include/${MODULE_NAME}/role_manager.hpp
)

target_link_libraries(${MODULE_NAME}
    PRIVATE
    game_data_provider
    marker_service
    PUBLIC
    time
    logger
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)