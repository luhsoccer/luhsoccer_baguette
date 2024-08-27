set(MODULE_NAME
    core
)

add_library(${MODULE_NAME} STATIC
    include/${MODULE_NAME}/common_types.hpp
    include/${MODULE_NAME}/module.hpp
    include/${MODULE_NAME}/robot_identifier.hpp
    include/${MODULE_NAME}/visit.hpp
    src/core.cpp
)

find_package(fmt REQUIRED)

target_link_libraries(${MODULE_NAME}
    PUBLIC
    event_system
    PRIVATE
    fmt::fmt
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)