set(MODULE_NAME
    utils
)

add_library(${MODULE_NAME} STATIC
    src/utils.cpp
    src/luts.cpp
    include/${MODULE_NAME}/utils.hpp
    include/${MODULE_NAME}/luts.hpp
)

#find_package(MyLib REQUIRED)

target_link_libraries(${MODULE_NAME}
    logger
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)