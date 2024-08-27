set(MODULE_NAME
    logger
)

add_library(${MODULE_NAME} STATIC
    src/logger.cpp
    src/gui_callback.cpp
    include/${MODULE_NAME}/logger.hpp
    include/${MODULE_NAME}/gui_callback.hpp
)

find_package(spdlog REQUIRED)
find_package(MPMCQueue REQUIRED)

target_link_libraries(${MODULE_NAME}
    PUBLIC
    fmt::fmt
    PRIVATE
    MPMCQueue::MPMCQueue
    spdlog::spdlog
    utils
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)