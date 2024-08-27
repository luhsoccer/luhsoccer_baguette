set(MODULE_NAME
    robot_control
)

add_library(${MODULE_NAME} STATIC
    src/components/steps/drive_step_constraint.hpp
    src/components/steps/drive_step_constraints/stop_state_constraint.hpp
    src/components/steps/drive_step_constraints/stop_state_constraint.cpp
    src/components/steps/drive_step_constraints/defense_area.cpp
    src/components/steps/drive_step_constraints/defense_area.hpp
    src/components/steps/drive_step_constraints/avoid_other_robots.hpp
    src/components/steps/drive_step_constraints/avoid_other_robots.cpp
    src/components/steps/drive_step_constraints/boundaries.cpp
    src/components/steps/drive_step_constraints/boundaries.hpp
    src/components/steps/drive_step_constraints/enemy_kickoff.cpp
    src/components/steps/drive_step_constraints/enemy_kickoff.hpp
    src/components/steps/drive_step_constraints/leave_field.cpp
    src/components/steps/drive_step_constraints/leave_field.hpp
    src/components/steps/drive_step_constraints/ball_placement.cpp
    src/components/steps/drive_step_constraints/ball_placement.hpp

    src/components/steps/condition_step.cpp
    src/components/steps/dribbler_step.cpp
    src/components/steps/kick_step.cpp
    src/components/steps/drive_step_initialization.cpp
    src/components/steps/drive_step_implementation.cpp
    src/components/steps/turn_around_ball_step.cpp
    src/components/steps/wait_step.cpp

    src/components/abstract_feature.cpp
    src/components/component_uid.cpp
    src/components/component_util.cpp
    
    src/components/features/target_feature.cpp
    src/components/features/anti_target_feature.cpp
    src/components/features/turn_around_ball_feature.cpp
    src/components/features/obstacle_feature.cpp

    src/components/rotation_controls/heading_rotation_control.cpp
    src/components/rotation_controls/rotate_to_move_direction_control.cpp

    src/components/shapes/point_shape.cpp
    src/components/shapes/arc_shape.cpp
    src/components/shapes/circle_shape.cpp
    src/components/shapes/compose_shape.cpp
    src/components/shapes/line_shape.cpp
    src/components/shapes/rectangle_shape.cpp
    
    src/cooperation_module.hpp
    src/cooperation_module.cpp
    src/marker_adapter.cpp
    src/marker_adapter.hpp
    src/robot_control_module.cpp
    src/robot_controller.cpp
    src/robot_controller.hpp
    src/skills/skill.cpp
    src/skills/task_data.cpp

    src/robot_dynamic.cpp
    src/robot_dynamic.hpp
    src/simulator.cpp
    src/simulator.hpp
    src/simulation_manager.cpp
    src/simulation_manager.hpp
    src/color_util.hpp


    include/${MODULE_NAME}/components/abstract_component.hpp
    include/${MODULE_NAME}/components/abstract_feature.hpp
    include/${MODULE_NAME}/components/abstract_rotation_control.hpp
    include/${MODULE_NAME}/components/abstract_shape.hpp
    include/${MODULE_NAME}/components/abstract_step.hpp
    include/${MODULE_NAME}/components/component_uid.hpp
    include/${MODULE_NAME}/components/component_util.hpp
    include/${MODULE_NAME}/components/component_data.hpp

    include/${MODULE_NAME}/components/features/target_feature.hpp
    include/${MODULE_NAME}/components/features/anti_target_feature.hpp
    include/${MODULE_NAME}/components/features/obstacle_feature.hpp
    include/${MODULE_NAME}/components/features/turn_around_ball_feature.hpp

    include/${MODULE_NAME}/components/rotation_controls/heading_rotation_control.hpp
    include/${MODULE_NAME}/components/rotation_controls/rotate_to_move_direction_control.hpp

    include/${MODULE_NAME}/components/shapes/point_shape.hpp
    include/${MODULE_NAME}/components/shapes/arc_shape.hpp
    include/${MODULE_NAME}/components/shapes/circle_shape.hpp
    include/${MODULE_NAME}/components/shapes/compose_shape.hpp
    include/${MODULE_NAME}/components/shapes/line_shape.hpp
    include/${MODULE_NAME}/components/shapes/rectangle_shape.hpp
    include/${MODULE_NAME}/components/shapes/inverse_rectangle_shape.hpp
    

    include/${MODULE_NAME}/components/steps/condition_step.hpp
    include/${MODULE_NAME}/components/steps/dribbler_step.hpp
    include/${MODULE_NAME}/components/steps/kick_step.hpp
    include/${MODULE_NAME}/components/steps/drive_step.hpp
    include/${MODULE_NAME}/components/steps/turn_around_ball_step.hpp
    include/${MODULE_NAME}/components/steps/wait_step.hpp


    include/${MODULE_NAME}/robot_control_module.hpp
    include/${MODULE_NAME}/robot_controller_state.hpp
    include/${MODULE_NAME}/skills/skill.hpp
    include/${MODULE_NAME}/skills/task_data.hpp



)

#find_package(MyLib REQUIRED)

target_link_libraries(${MODULE_NAME}
    robot_interface
    config_provider
    marker_service
    transform
    time
    utils
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)