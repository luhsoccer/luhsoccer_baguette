set(MODULE_NAME
    ssl_interface
)

add_library(${MODULE_NAME} STATIC
    src/vision_multicast/vision_multicast.cpp
    src/vision_multicast/vision_multicast.hpp
    src/gc_multicast/gc_multicast.cpp
    src/gc_multicast/gc_multicast.hpp
    src/gc_tcp/gc_tcp.cpp
    src/gc_tcp/gc_tcp.hpp
    src/log_file/log_file.cpp
    src/log_file/log_file_parser.cpp
    src/log_file/log_file_parser.hpp
    src/ssl_interface.cpp
    src/ssl_types_converter/ssl_types_converter.cpp
    src/ssl_types_converter/ssl_types_converter.hpp
    include/${MODULE_NAME}/ssl_interface.hpp
    include/${MODULE_NAME}/log_file.hpp)

# Include the protobuf files
target_include_directories(
    ${MODULE_NAME} PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}
)

find_package(Eigen3 REQUIRED)
find_package(asio REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Boost COMPONENTS iostreams REQUIRED)

# Disable unity builds for protobuf files since they are generated and violate ODRs
set_target_properties(luhsoccer_baguette_protobuf PROPERTIES UNITY_BUILD FALSE)

target_link_libraries(${MODULE_NAME}
    PRIVATE
    asio::asio
    config_provider
    luhsoccer_baguette_protobuf
    simulation_interface
    marker_service
    Boost::boost
    Boost::iostreams
    PUBLIC
    ZLIB::ZLIB
    logger
    Eigen3::Eigen
    time
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)