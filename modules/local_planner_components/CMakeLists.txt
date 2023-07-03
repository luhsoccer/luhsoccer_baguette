set(MODULE_NAME
    local_planner_components
)

add_library(${MODULE_NAME} STATIC
    src/shapes/point_shape.cpp
    include/${MODULE_NAME}/shapes/point_shape.hpp

    src/shapes/circle_shape.cpp
    include/${MODULE_NAME}/shapes/circle_shape.hpp

    src/shapes/line_shape.cpp
    include/${MODULE_NAME}/shapes/line_shape.hpp

    src/shapes/rectangle_shape.cpp
    include/${MODULE_NAME}/shapes/rectangle_shape.hpp

    src/shapes/compose_shape.cpp
    include/${MODULE_NAME}/shapes/compose_shape.hpp

    src/shapes/arc_shape.cpp
    include/${MODULE_NAME}/shapes/arc_shape.hpp

    src/features/target_feature.cpp
    src/features/anti_target_feature.cpp
    src/features/ball_target_feature.cpp
    src/features/turn_around_ball_feature.cpp
    src/features/robot_cf_obstacle.cpp
    include/${MODULE_NAME}/features/target_feature.hpp
    include/${MODULE_NAME}/features/anti_target_feature.hpp
    include/${MODULE_NAME}/features/ball_target_feature.hpp
    include/${MODULE_NAME}/features/turn_around_ball_feature.hpp
    include/${MODULE_NAME}/features/robot_cf_obstacle.hpp

    src/rotation_controls/heading_rotation_control.cpp
    src/rotation_controls/rotate_to_move_direction_control.cpp
    include/${MODULE_NAME}/rotation_controls/heading_rotation_control.hpp
    include/${MODULE_NAME}/rotation_controls/rotate_to_move_direction_control.hpp

    
    include/${MODULE_NAME}/steps/drive_step_constraint.hpp
    src/steps/drive_step_constraints/drive_step_constraint.cpp
    src/steps/drive_step_constraints/avoid_other_robots.hpp
    src/steps/drive_step_constraints/avoid_other_robots.cpp
    src/steps/drive_step_constraints/defense_area.hpp
    src/steps/drive_step_constraints/defense_area.cpp
    src/steps/drive_step_constraints/stop_state.hpp
    src/steps/drive_step_constraints/stop_state.cpp
    src/steps/drive_step_constraints/kickoff.hpp
    src/steps/drive_step_constraints/kickoff.cpp
    src/steps/drive_step_constraints/ball_placement.cpp
    src/steps/drive_step_constraints/ball_placement.hpp

    src/steps/drive_step.cpp
    src/steps/wait_step.cpp
    src/steps/kick_step.cpp
    src/steps/turn_around_ball_step.cpp
    include/${MODULE_NAME}/steps/drive_step.hpp
    include/${MODULE_NAME}/steps/wait_step.hpp
    include/${MODULE_NAME}/steps/kick_step.hpp
    include/${MODULE_NAME}/steps/turn_around_ball_step.hpp

    src/steps/condition_step.cpp
    include/${MODULE_NAME}/steps/condition_step.hpp

    src/steps/dribbler_step.cpp
    include/${MODULE_NAME}/steps/dribbler_step.hpp

    include/${MODULE_NAME}/complex_positions.hpp

    )

#find_package(MyLib REQUIRED)

target_link_libraries(${MODULE_NAME}
    local_planner
    config_provider
    utils
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)