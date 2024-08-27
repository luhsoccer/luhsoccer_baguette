set(MODULE_NAME
    pch
)

add_library(${MODULE_NAME} OBJECT
    src/pch.cpp
)

target_precompile_headers(pch
    PUBLIC
    include/pch/pch.hpp
)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(tomlplusplus REQUIRED IMPORTED_TARGET tomlplusplus)


find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

target_link_libraries(
    ${MODULE_NAME}
    PUBLIC
    Eigen3::Eigen
    fmt::fmt
    PkgConfig::tomlplusplus
)

target_include_directories(
    ${MODULE_NAME}
    PUBLIC include
)