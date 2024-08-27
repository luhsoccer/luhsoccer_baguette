set(MODULE_NAME
    event_system
)

add_library(${MODULE_NAME} STATIC
    src/event_system.cpp
    src/event_system_helper.cpp
    include/${MODULE_NAME}/event_system.hpp
    include/${MODULE_NAME}/event_system_helper.hpp
    include/${MODULE_NAME}/event.hpp
    include/${MODULE_NAME}/timer_events.hpp
)

find_package(asio REQUIRED)
find_path(BSHOSHANY_THREAD_POOL_INCLUDE_DIRS "BS_thread_pool.hpp")

target_link_libraries(${MODULE_NAME}
    PUBLIC
    logger
    time
    PRIVATE
    utils
    asio::asio
)

target_include_directories(${MODULE_NAME}
    PUBLIC ${BSHOSHANY_THREAD_POOL_INCLUDE_DIRS}
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)