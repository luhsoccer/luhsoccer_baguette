configure_file(../include/msdfgl.h.in msdfgl.h @ONLY)

# Finds all .glsl shaders in our source directory.
file(GLOB_RECURSE msdfgl_shaders ${CMAKE_CURRENT_SOURCE_DIR}/*.glsl)

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/_msdfgl_shaders.h
                   DEPENDS ${msdfgl_shaders}
                   COMMAND ${CMAKE_COMMAND} -DSHADER_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
                                            -DTARGET_DIR="${CMAKE_CURRENT_BINARY_DIR}"
                                            -P "${CMAKE_CURRENT_SOURCE_DIR}/msdfgl_shaders.cmake")

find_package(Freetype REQUIRED)

add_library(msdfgl STATIC ../include/msdfgl.h msdfgl.c msdfgl_serializer.c msdfgl_map.c
            ${CMAKE_CURRENT_BINARY_DIR}/_msdfgl_shaders.h)

target_compile_definitions(msdfgl PUBLIC MSDFGL_STATIC_DEFINE)

target_include_directories(msdfgl PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
                           ${FREETYPE_INCLUDE_DIRS})


find_package(glad REQUIRED)
target_link_libraries(msdfgl PUBLIC glad::glad Freetype::Freetype)

target_compile_features(msdfgl PUBLIC c_std_11)
if(MSVC)
    #target_compile_options(msdfgl PRIVATE /W4 /WX)
elseif (APPLE)
  set_target_properties(msdfgl PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
  target_compile_options(msdfgl PRIVATE -Wall -Wextra -pedantic -Werror
                         -Wno-overlength-strings -Wno-unused-parameter -Wno-unused-variable
                         -DGL_SILENCE_DEPRECATION)
else()
  target_compile_options(msdfgl PRIVATE -Wall -Wextra -pedantic -Werror
                         -Wno-overlength-strings -Wno-unused-parameter -Wno-unused-variable)
endif()