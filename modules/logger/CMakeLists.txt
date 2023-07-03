set(MODULE_NAME
    logger
)

add_library(${MODULE_NAME} STATIC
    src/logger.cpp
    include/${MODULE_NAME}/logger.hpp
    include/${MODULE_NAME}/gui_sink.hpp)

find_package(spdlog REQUIRED)

target_link_libraries(${MODULE_NAME} PUBLIC
    spdlog::spdlog
    MPMCQueue::MPMCQueue
    PRIVATE
    utils
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)