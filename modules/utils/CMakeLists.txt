set(MODULE_NAME
    utils
)

add_library(${MODULE_NAME} STATIC
    src/utils.cpp
    src/luts.cpp
    include/${MODULE_NAME}/utils.hpp
    include/${MODULE_NAME}/luts.hpp
)

find_path(PORTABLE_FILE_DIALOGS_INCLUDE_DIRS "portable-file-dialogs.h")

# Disable unity build for this module, since it includes Windows.h which is incompatible with unity build
set_target_properties(${MODULE_NAME} PROPERTIES UNITY_BUILD FALSE)

target_include_directories(${MODULE_NAME}
    PRIVATE 
    ${PORTABLE_FILE_DIALOGS_INCLUDE_DIRS}
)

target_link_libraries(${MODULE_NAME}
    PUBLIC
    core
    logger
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)