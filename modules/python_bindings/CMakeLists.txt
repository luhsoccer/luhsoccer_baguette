set(MODULE_NAME
    _baguette_py
)

if(BUILD_PYTHON_MODULE)
    message("Building python module")
    find_package(Python3 COMPONENTS Interpreter Development.Module REQUIRED)
else()
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
endif()

find_package(pybind11 REQUIRED)

pybind11_add_module(_baguette_py
    src/python_module.cpp
    src/module_bindings.cpp
    src/tool_bindings.cpp
    src/bindings/common.cpp
    src/bindings/game_data_provider.cpp
    src/bindings/config_provider.cpp
    src/bindings/role_manager.cpp
    src/bindings/ssl_interface.cpp
    src/bindings/time.cpp
    src/bindings/transform.cpp
    src/bindings/marker_service.cpp
    src/bindings/robot_interface.cpp
    src/bindings/simulation_interface.cpp
    src/bindings/skills.cpp
    src/bindings/local_planner.cpp
    src/bindings/task_manager.cpp
    src/bindings/util.cpp
)

if(BUILD_PYTHON_MODULE)
    target_link_libraries(${MODULE_NAME} PRIVATE pybind11::headers Python3::Module)
#    if(MSVC)
#        set_property(TARGET ${MODULE_NAME} PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
#    endif()
else()
    target_link_libraries(${MODULE_NAME} PRIVATE pybind11::headers ${Python3_LIBRARIES})
endif()

# Add pybind to all modules since it is used by the common module header
target_include_directories(${MODULE_NAME} PRIVATE ${Python3_INCLUDE_DIRS})


target_link_libraries(${MODULE_NAME} PUBLIC
    baguette_static
    pybind11::module
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

if(BUILD_PYTHON_MODULE)
    if(MSVC)
        install(FILES $<TARGET_PDB_FILE:${MODULE_NAME}> DESTINATION baguette_py OPTIONAL)
    endif()
    install(TARGETS ${MODULE_NAME} DESTINATION baguette_py)
#    install(CODE "execute_process(COMMAND ${CMAKE_SOURCE_DIR}/stubs/generate_stubs.sh \${CMAKE_INSTALL_PREFIX}/baguette_py RESULT_VARIABLE stub_result OUTPUT_VARIABLE stub_output ERROR_VARIABLE stub_error)")
#    install(CODE "message(\"Generate stubs [\${stub_result}]: \${stub_output} \${stub_error}\")")
endif()

# Can't enable pch here since we're building a python module
#target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)