set(MODULE_NAME
    experiment_logger
)

add_library(${MODULE_NAME} STATIC
    src/experiment_logger.cpp
    include/${MODULE_NAME}/experiment_logger.hpp)

#find_package(MyLib REQUIRED)

target_link_libraries(${MODULE_NAME}
    transform
    marker_service
    utils
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)