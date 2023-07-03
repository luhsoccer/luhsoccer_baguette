set(MODULE_NAME
    exception_handler
)

add_library(${MODULE_NAME} STATIC
    include/${MODULE_NAME}/exception_handler.hpp
    src/exception_handler.cpp
)

target_include_directories(${MODULE_NAME}
    PRIVATE src
    PUBLIC include
)

find_package(Boost REQUIRED)

set(
    MODULES_TO_LINK

    utils 
    logger
    baguette_static
    Boost::boost
)

# MSVC doesnt need libbacktrace
if(NOT MSVC)
    message("Adding libbacktrace")
    find_package(libbacktrace REQUIRED)
    list(
        APPEND
        MODULES_TO_LINK
        
        libbacktrace::libbacktrace
   )
endif()


target_link_libraries(${MODULE_NAME} 
    
    PRIVATE
    ${MODULES_TO_LINK}
)



target_precompile_headers(${MODULE_NAME} REUSE_FROM pch)