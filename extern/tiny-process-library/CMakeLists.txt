cmake_minimum_required(VERSION 3.1)

project(tiny-process-library)

add_library(tiny-process-library process.cpp)
add_library(tiny-process-library::tiny-process-library ALIAS tiny-process-library)

if(MSVC)
  target_compile_definitions(tiny-process-library PRIVATE /D_CRT_SECURE_NO_WARNINGS)
else()
  target_compile_options(tiny-process-library PRIVATE -std=c++11 -Wall -Wextra)
endif()

if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  target_sources(tiny-process-library PRIVATE process_win.cpp)
  #If compiled using MSYS2, use sh to run commands
  if(MSYS)
    target_compile_definitions(tiny-process-library PUBLIC MSYS_PROCESS_USE_SH)
  endif()
else()
  target_sources(tiny-process-library PRIVATE process_unix.cpp)
endif()

find_package(Threads REQUIRED)

target_link_libraries(tiny-process-library ${CMAKE_THREAD_LIBS_INIT})
target_include_directories(tiny-process-library PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                                       $<INSTALL_INTERFACE:include>)
