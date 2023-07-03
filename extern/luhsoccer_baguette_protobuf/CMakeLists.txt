set(PROTOBUF_USE_STATIC_LIBS ON)


find_package(Protobuf REQUIRED)

set(Protobuf_IMPORT_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/erforce" 
    "${CMAKE_CURRENT_SOURCE_DIR}/luhsoccer"
    "${CMAKE_CURRENT_SOURCE_DIR}/ssl_autoref"
    "${CMAKE_CURRENT_SOURCE_DIR}/ssl_gc"
    "${CMAKE_CURRENT_SOURCE_DIR}/ssl_simulation"
    "${CMAKE_CURRENT_SOURCE_DIR}/ssl_vision")

set(ERFORCE_PROTO_FILES
    erforce/ssl_simulation_custom_erforce_realism.proto
    erforce/ssl_simulation_custom_erforce_robot_spec.proto
)

set(LUHSOCCER_PROTO_FILES
    luhsoccer/luhsoccer_simulation_control.proto
    luhsoccer/luhsoccer_robot_interface.proto
    luhsoccer/luhsoccer_basestation.proto
    luhsoccer/luhsoccer_luhviz_markers.proto
)

set(SSL_AUTOREF_PROTO_FILES
    ssl_autoref/ssl_autoref_ci.proto
)

set(SSL_GC_PROTO_FILES
    ssl_gc/ssl_gc_api.proto
    ssl_gc/ssl_gc_change.proto
    ssl_gc/ssl_gc_ci.proto
    ssl_gc/ssl_gc_common.proto
    ssl_gc/ssl_gc_engine_config.proto
    ssl_gc/ssl_gc_engine.proto
    ssl_gc/ssl_gc_game_event.proto
    ssl_gc/ssl_gc_geometry.proto
    ssl_gc/ssl_gc_rcon_autoref.proto
    ssl_gc/ssl_gc_rcon_remotecontrol.proto
    ssl_gc/ssl_gc_rcon_team.proto
    ssl_gc/ssl_gc_rcon.proto
    ssl_gc/ssl_gc_referee_message.proto
    ssl_gc/ssl_gc_state.proto
)

set(SSL_SIMULATION_PROTO_FILES
    ssl_simulation/ssl_simulation_config.proto
    ssl_simulation/ssl_simulation_control.proto
    ssl_simulation/ssl_simulation_error.proto
    ssl_simulation/ssl_simulation_robot_control.proto
    ssl_simulation/ssl_simulation_robot_feedback.proto
    ssl_simulation/ssl_simulation_synchronous.proto
)

set(SSL_VISION_PROTO_FILES
    ssl_vision/ssl_vision_detection_tracked.proto
    ssl_vision/ssl_vision_detection.proto
    ssl_vision/ssl_vision_geometry.proto
    ssl_vision/ssl_vision_wrapper_tracked.proto
    ssl_vision/ssl_vision_wrapper.proto
)

protobuf_generate_cpp(
    PROTO_SOURCES
    PROTO_HEADERS
    ${ERFORCE_PROTO_FILES}
    ${LUHSOCCER_PROTO_FILES}
    ${SSL_AUTOREF_PROTO_FILES} 
    ${SSL_GC_PROTO_FILES}
    ${SSL_SIMULATION_PROTO_FILES}
    ${SSL_VISION_PROTO_FILES}
)

add_library(luhsoccer_baguette_protobuf STATIC)
target_sources(luhsoccer_baguette_protobuf PRIVATE ${PROTO_SOURCES} ${PROTO_HEADERS} ${PROTO_FILES})

target_compile_options(luhsoccer_baguette_protobuf PUBLIC -w)

target_link_libraries(luhsoccer_baguette_protobuf
    PUBLIC
    protobuf::libprotobuf
)

target_include_directories(luhsoccer_baguette_protobuf
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
)
target_include_directories(luhsoccer_baguette_protobuf SYSTEM
    PUBLIC ${PROTOBUF_INCLUDE_DIR}
    PRIVATE ${PROTOBUF_INCLUDE_DIR}
)