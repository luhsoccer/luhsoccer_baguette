set(MODULE_NAME
    vision_processor
)

add_library(${MODULE_NAME} STATIC
    src/vision_processor.cpp
    include/${MODULE_NAME}/vision_processor.hpp
)

#find_package(MyLib REQUIRED)

target_link_libraries(${MODULE_NAME}
    PRIVATE
    ssl_interface
    
    PUBLIC
    core
    
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)