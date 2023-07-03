set(MODULE_NAME
    robot_interface
)

add_library(${MODULE_NAME} STATIC
    src/robot_interface.cpp
    src/robot_interface_types.cpp
    src/connections/serial.hpp
    src/connections/serial.cpp
    src/connections/network.hpp
    src/connections/network.cpp
    src/connections/simulation.hpp
    src/connections/simulation.cpp
    include/${MODULE_NAME}/robot_interface.hpp
    include/${MODULE_NAME}/robot_interface_types.hpp
)

find_package(Eigen3 REQUIRED)
find_package(serial REQUIRED)
find_package(asio REQUIRED)

target_link_libraries(${MODULE_NAME}
    PRIVATE
    config_provider
    simulation_interface
    luhsoccer_baguette_protobuf
    serial::serial
    asio::asio
    PUBLIC
    Eigen3::Eigen
    transform
    logger
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)