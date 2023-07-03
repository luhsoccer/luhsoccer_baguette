set(MODULE_NAME
    time
)

add_library(${MODULE_NAME} STATIC
    src/time.cpp
    src/calc_time_stopwatch.cpp
    include/${MODULE_NAME}/time.hpp
    include/${MODULE_NAME}/calc_time_stopwatch.hpp
    )

target_link_libraries(${MODULE_NAME}
    logger
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)