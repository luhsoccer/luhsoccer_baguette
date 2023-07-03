set(MODULE_NAME
    marker_service
)

add_library(${MODULE_NAME} STATIC
    src/marker_service.cpp
    src/marker_2d_impl.cpp
    src/marker_test.cpp
    src/marker.cpp
    include/${MODULE_NAME}/marker_test.hpp
    include/${MODULE_NAME}/marker.hpp
    include/${MODULE_NAME}/marker_impl.hpp
    include/${MODULE_NAME}/marker_2d_impl.hpp
    include/${MODULE_NAME}/marker_service.hpp)

find_package(Eigen3 REQUIRED)
find_package(glm REQUIRED)

target_link_libraries(${MODULE_NAME}
    Eigen3::Eigen
    logger
    time
    transform
    glm::glm
    game_data_provider
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)