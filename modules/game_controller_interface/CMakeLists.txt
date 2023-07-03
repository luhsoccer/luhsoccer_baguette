set(MODULE_NAME
    game_controller_interface
)

add_library(${MODULE_NAME} STATIC

    include/${MODULE_NAME}/game_controller_interface.hpp 
    src/game_controller_interface.cpp
    src/game_controller_interface_impl.hpp
    src/game_controller_interface_impl.cpp
    
)

find_package(Eigen3 REQUIRED)
find_package(tomlplusplus REQUIRED)

target_link_libraries(${MODULE_NAME}
    PRIVATE
    tiny-process-library
    logger
    time
    PUBLIC
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)