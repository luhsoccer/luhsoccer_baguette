set(MODULE_NAME
    __template
)

add_library(${MODULE_NAME} STATIC
    src/__template.cpp
    include/${MODULE_NAME}/__template.hpp
)

#find_package(MyLib REQUIRED)

#target_link_libraries(${MODULE_NAME}
    #MyLibDepdencies
    #MyModuleDependencies
#)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)