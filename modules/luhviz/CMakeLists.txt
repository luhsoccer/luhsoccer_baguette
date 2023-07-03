set(MODULE_NAME
    luhviz
)

add_library(${MODULE_NAME} STATIC
    # cpp
    src/debugger/debugger.cpp 
    src/game_info/game_info.cpp
    src/inspector/inspector.cpp
    src/main_window/main_window.cpp
    src/render_view/render_view.cpp
    src/render_view/model.cpp
    src/luhconfig/luhconfig.cpp
    src/common/controls.cpp
    src/common/physics.cpp
    src/luhviz.cpp
    src/data_proxy.cpp
    src/skill_tester/skill_tester.cpp
    src/timeline/timeline.cpp
    src/robot_controller/robot_controller.cpp
    src/skill_wizard/skill_wizard.cpp
    src/robot_info_display/robot_info_display.cpp
    src/info_display/info_display.cpp
    src/imgui_backend/imgui_impl_glfw.cpp
    src/imgui_backend/imgui_impl_opengl3.cpp
    src/imgui_backend/imgui_stdlib.cpp
    src/imgui_backend/implot_demo.cpp
    src/plotter/plotter.cpp
    src/new_rendering/gl_context.cpp
    src/new_rendering/gl_shader.cpp
    src/new_rendering/gl_shader_program.cpp
    src/new_rendering/gl_texture.cpp
    src/new_rendering/gl_vertex_array.cpp
    src/new_rendering/gl_buffer.cpp
    src/new_rendering/gl_uniform.cpp
    src/game_controller_window/game_controller_window.cpp
    # hpp
    src/debugger/include/debugger.hpp
    src/debugger/include/debugger_event.hpp
    src/game_info/include/game_info.hpp
    src/inspector/include/inspector.hpp
    src/main_window/include/main_window.hpp
    src/render_view/include/render_view.hpp
    src/render_view/include/mesh.hpp
    src/render_view/include/model.hpp
    src/luhconfig/include/luhconfig.hpp
    src/common/include/controls.hpp
    src/common/include/utils.hpp
    src/common/include/fonts.hpp
    src/common/include/physics.hpp
    src/include/data_proxy.hpp
    src/include/data_structs.hpp
    src/skill_tester/include/skill_tester.hpp
    src/timeline/include/timeline.hpp
    src/robot_controller/include/robot_controller.hpp
    src/skill_wizard/include/skill_wizard.hpp
    src/robot_info_display/include/robot_info_display.hpp
    src/info_display/include/info_display.hpp
    src/imgui_backend/imgui_impl_glfw.h
    src/imgui_backend/imgui_impl_opengl3.h
    src/imgui_backend/imgui_stdlib.h
    src/new_rendering/include/gl_context.hpp
    src/new_rendering/include/gl_shader.hpp
    src/new_rendering/include/gl_shader_program.hpp
    src/new_rendering/include/gl_texture.hpp
    src/new_rendering/include/gl_vertex_array.hpp
    src/new_rendering/include/gl_buffer.hpp
    src/new_rendering/include/gl_uniform.hpp
    src/plotter/include/plotter.hpp
    src/game_controller_window/include/game_controller_window.hpp
    # include
    include/${MODULE_NAME}/luhviz.hpp)

find_package(imgui REQUIRED)
find_package(glfw3 REQUIRED)
find_package(glad REQUIRED)
find_package(glm REQUIRED)
find_package(stb REQUIRED)
find_package(tinyobjloader REQUIRED)
find_package(implot REQUIRED)
find_package(portable-file-dialogs REQUIRED)

target_link_libraries(${MODULE_NAME}
    PRIVATE
    imgui::imgui
    glfw::glfw
    glad::glad
    glm::glm
    stb::stb
    tinyobjloader::tinyobjloader
    portable-file-dialogs::portable-file-dialogs
    implot::implot
    logger
    time
    transform
    marker_service
    ssl_interface
    game_data_provider
    luhviz_ressources
    config_provider
    simulation_interface
    local_planner
    skills
    robot_interface
    scenario
    msdfgl
    game_controller_interface
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)