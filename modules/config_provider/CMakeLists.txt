set(MODULE_NAME
    config_provider
)

add_library(${MODULE_NAME} STATIC

    src/config_provider.cpp
    src/config_store.cpp
    src/utility.cpp


    include/${MODULE_NAME}/basic_container.hpp 
    include/${MODULE_NAME}/datatypes.hpp
    include/${MODULE_NAME}/parameters.hpp
    include/${MODULE_NAME}/config_base.hpp
    include/${MODULE_NAME}/utility.hpp

    include/config/config_store.hpp
    include/config/observer_config.hpp
    
)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(tomlplusplus REQUIRED IMPORTED_TARGET tomlplusplus)

target_link_libraries(${MODULE_NAME}
    PRIVATE
    config_provider_resources
    PUBLIC
    Eigen3::Eigen
    PkgConfig::tomlplusplus
    logger
    utils
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

# Can't be enabled here because tomlplusplus needs to be recompiled
# target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)