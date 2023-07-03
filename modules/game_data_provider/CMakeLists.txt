set(MODULE_NAME
    game_data_provider
)

add_library(${MODULE_NAME} STATIC
 
    # Game Data Provider 
    src/game_data_provider.cpp
    src/robot_data_filter.cpp
    src/ball_filter/ball_filter.cpp
    src/ball_filter/static_queue.hpp
    src/ball_filter/speed_filter.cpp
    src/ball_filter/speed_filter.hpp
    src/ball_filter/moving_average_filter.cpp
    src/ball_filter/moving_average_filter.hpp
    src/ball_filter/base_filter.cpp
    src/ball_filter/base_filter.hpp
    include/${MODULE_NAME}/game_data_provider.hpp
    include/${MODULE_NAME}/team_info.hpp
    include/${MODULE_NAME}/team_info.hpp

    # Static Observer stuff
    include/observer/static_observer.hpp
    src/observer/static_observer.cpp
    include/observer/static_observer_position.hpp
    src/observer/static_observer_position.cpp
    
    src/observer/pass_probability/pass_probability.hpp
    src/observer/pass_probability/pass_probability.cpp

    src/observer/goal_probability/goal_probability.cpp
    src/observer/goal_probability/goal_probability.hpp

    # Continuous Observer stuff
    include/observer/buffer.hpp
    include/observer/continuous_observer.hpp
    include/observer/data_storage.hpp
    include/observer/robot_datatypes.hpp

    src/observer/continuous_observer/continuous_observer.cpp
    src/observer/continuous_observer/observer_getter.cpp
    src/observer/data_storage/data_storage_getter.cpp
    src/observer/data_storage/data_storage_setter.cpp

    src/data_processor.hpp
 
    # Observer misc
    src/observer/misc/misc.cpp
    src/observer/misc/misc.hpp
    src/observer/utility.hpp
)

find_package(Eigen3 REQUIRED)

target_link_libraries(${MODULE_NAME}
    PRIVATE
    ssl_interface
    robot_interface
    utils
    marker_service
    config_provider
    robot_interface
    PUBLIC
    transform
    logger
    Eigen3::Eigen
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)