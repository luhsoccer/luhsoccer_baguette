set(MODULE_NAME
    transform
)

add_library(${MODULE_NAME} STATIC
    src/transform.cpp
    src/position.cpp
    src/world_model.cpp
    include/${MODULE_NAME}/circular_buffer.hpp
    include/${MODULE_NAME}/transform.hpp
    include/${MODULE_NAME}/position.hpp
    include/${MODULE_NAME}/world_model.hpp
    include/${MODULE_NAME}/game_state.hpp

    include/transform_helper/world_model_helper.hpp
    src/world_model_helper.cpp

    )

find_package(Eigen3 REQUIRED)

target_link_libraries(${MODULE_NAME}
    time
    Eigen3::Eigen
    logger
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)