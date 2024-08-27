set(MODULE_NAME
    simulation_interface
)

add_library(${MODULE_NAME} STATIC
    include/${MODULE_NAME}/simulation_interface.hpp
    src/simulation_interface.cpp
    include/${MODULE_NAME}/simulation_connector.hpp
    src/simulation_connector.cpp
    src/simulations/test_simulation_connector.hpp
    src/simulations/erforce_simulation_connector.hpp
    src/simulations/erforce_simulation_connector.cpp
    src/simulations/er_sim_connector.cpp
    src/simulations/er_sim_connector.hpp
)

    
find_package(asio REQUIRED)
find_package(Eigen3 REQUIRED)

target_link_libraries(${MODULE_NAME} 
    PRIVATE
    asio::asio
    luhsoccer_baguette_protobuf
    config_provider
    vision_processor
    PUBLIC
    event_system
    Eigen3::Eigen
    logger
    time
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)