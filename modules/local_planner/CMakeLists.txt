set(MODULE_NAME
    local_planner
)

add_library(${MODULE_NAME} STATIC
    src/agent.cpp
    src/local_planner.cpp
    src/local_planner_module.cpp
    src/robot_dynamic.cpp
    src/thread_pool.cpp
    src/simulator.cpp
    src/simulation_manager.cpp
    src/avoidance_manager.cpp
    src/avoidance_manager_unit.cpp

    src/skills/skill.cpp
    src/skills/abstract_feature.cpp
    src/skills/skill_util.cpp
    src/skills/component_uid.cpp
    src/skills/task.cpp

    
    include/${MODULE_NAME}/skills/abstract_shape.hpp
    include/${MODULE_NAME}/skills/abstract_feature.hpp
    include/${MODULE_NAME}/skills/abstract_rotation_control.hpp
    include/${MODULE_NAME}/skills/abstract_step.hpp
    include/${MODULE_NAME}/skills/abstract_component.hpp
    include/${MODULE_NAME}/skills/skill.hpp
    include/${MODULE_NAME}/skills/skill_util.hpp
    include/${MODULE_NAME}/skills/task.hpp
    include/${MODULE_NAME}/local_planner_util.hpp
    include/${MODULE_NAME}/robot_dynamic.hpp
    include/${MODULE_NAME}/agent.hpp
    include/${MODULE_NAME}/local_planner.hpp
    include/${MODULE_NAME}/local_planner_module.hpp
    include/${MODULE_NAME}/simulator.hpp
    include/${MODULE_NAME}/thread_pool.hpp
    include/${MODULE_NAME}/simulation_manager.hpp
    include/${MODULE_NAME}/avoidance_manager.hpp
    include/${MODULE_NAME}/avoidance_manager_unit.hpp

    src/avoid_force_callbacks/calc_cf_force.cpp
    src/avoid_force_callbacks/calc_cf_force.hpp
    src/avoid_force_callbacks/cf_force_callback.cpp
    src/avoid_force_callbacks/cf_force_callback.hpp
    src/avoid_force_callbacks/apf_callback.hpp
    src/avoid_force_callbacks/apf_callback.cpp
    src/avoid_force_callbacks/ataka_callback.cpp
    src/avoid_force_callbacks/ataka_callback.hpp
    src/avoid_force_callbacks/gyroscopic_callback.cpp
    src/avoid_force_callbacks/gyroscopic_callback.hpp
    src/avoidance_manager_units/cooperative_side_decision.cpp
    src/avoidance_manager_units/cooperative_side_decision.hpp
    src/avoidance_manager_units/haddadin.cpp
    src/avoidance_manager_units/haddadin.hpp
    src/avoidance_manager_units/side_decision.cpp
    src/avoidance_manager_units/side_decision.hpp
    )

#find_package(MyLib REQUIRED)

target_link_libraries(${MODULE_NAME} PUBLIC
    robot_interface
    config_provider
    marker_service
)


target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)
