set(MODULE_NAME
    software_manager
)

add_library(${MODULE_NAME} STATIC
    include/${MODULE_NAME}/software_manager.hpp
    src/software_manager.cpp
)

find_package(unofficial-curlpp CONFIG REQUIRED)


target_link_libraries(${MODULE_NAME}
    PRIVATE
    utils
    unofficial::curlpp::curlpp
    simulator_resources
    config_provider
    PUBLIC
    logger
    core
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)